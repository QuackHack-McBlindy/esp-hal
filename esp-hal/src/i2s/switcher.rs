//! I2S half-duplex switcher for sharing clock pins between RX and TX.
//!
//! This module provides a wrapper around [`I2s`] that allows you to build either
//! an [`I2sRx`] or an [`I2sTx`] from the same underlying peripheral and clock pins.
//! The switcher consumes itself when building a side, guaranteeing that only one
//! direction is active at any time.

use esp_hal::{
    dma::DmaDescriptor,
    gpio::{AnyPin, InputPin, OutputPin, interconnect::PeripheralInput},
    i2s::master::{I2s, I2sRx, I2sTx, Blocking},
};

/// A wrapper that holds an I2S instance and separate DMA descriptors for RX and TX.
///
/// The I2S instance **must** have its clock pins (BCLK, WS, optional MCLK) already
/// configured via [`I2s::with_bclk`], [`I2s::with_ws`], etc. The switcher will then
/// connect those pins to both the RX and TX internal signals, so they can be used
/// for either direction.
pub struct I2sSwitcher<'d> {
    i2s: I2s<'d, Blocking>,
    rx_descriptors: &'static mut [DmaDescriptor],
    tx_descriptors: &'static mut [DmaDescriptor],
}

impl<'d> I2sSwitcher<'d> {
    /// Creates a new switcher.
    ///
    /// # Arguments
    /// * `i2s` – An I2S instance with clock pins already attached (via `with_bclk`, `with_ws`, etc.)
    /// * `rx_descriptors` – DMA descriptors for the RX channel
    /// * `tx_descriptors` – DMA descriptors for the TX channel
    pub fn new(
        i2s: I2s<'d, Blocking>,
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Self {
        Self {
            i2s,
            rx_descriptors,
            tx_descriptors,
        }
    }

    /// Converts the switcher into an I2S receiver (RX) with the given data input pin.
    ///
    /// This consumes the switcher; the TX side is dropped. The clock pins (BCLK, WS, MCLK)
    /// are shared and will be configured for both directions, but only the RX unit will be active.
    pub fn into_rx(mut self, din: impl PeripheralInput<'d>) -> I2sRx<'d, Blocking> {
        self.init_shared_clocks();
        self.i2s.i2s_rx
            .with_din(din)
            .build(self.rx_descriptors)
    }

    /// Converts the switcher into an I2S transmitter (TX) with the given data output pin.
    ///
    /// This consumes the switcher; the RX side is dropped. The clock pins are shared
    /// and will be configured for both directions, but only the TX unit will be active.
    pub fn into_tx(mut self, dout: impl OutputPin<'d>) -> I2sTx<'d, Blocking> {
        self.init_shared_clocks();
        self.i2s.i2s_tx
            .with_dout(dout)
            .build(self.tx_descriptors)
    }

    /// Initialises the shared clock pins (BCLK, WS, MCLK) for both RX and TX.
    ///
    /// This method connects the stored BCLK and WS pins to both the TX output signals
    /// and the RX input signals, and also configures the MCLK output if present.
    /// It must be called before building either side.
    fn init_shared_clocks(&mut self) {
        // Connect BCLK and WS to both TX outputs and RX inputs
        if let (Some(bclk), Some(ws)) = (self.i2s.bclk.take(), self.i2s.ws.take()) {
            // Configure as outputs for TX (master mode)
            bclk.connect_peripheral_to_output(self.i2s.i2s_tx.i2s.bclk_signal());
            ws.connect_peripheral_to_output(self.i2s.i2s_tx.i2s.ws_signal());

            // Enable input buffers so RX can also receive the clocks
            bclk.set_input_enable(true);
            ws.set_input_enable(true);

            // Connect RX input signals to the same pins
            self.i2s.i2s_tx.i2s.bclk_rx_signal().connect_to(&bclk);
            self.i2s.i2s_tx.i2s.ws_rx_signal().connect_to(&ws);
        }

        // Handle MCLK if present (only for master mode)
        if let Some(mclk) = self.i2s.mclk.take() {
            #[cfg(not(esp32))]
            {
                mclk.set_output_enable(true);
                self.i2s.i2s_tx.i2s.mclk_signal().connect_to(&mclk);
            }
            #[cfg(esp32)]
            {
                use esp_hal::gpio::OutputSignal;
                use esp_hal::peripherals::IO_MUX;

                let clk_signal = mclk.signal();
                mclk.set_output_enable(true);

                // Determine the selector based on the I2S instance
                let selector = match self.i2s.i2s_tx.i2s.0 {
                    esp_hal::i2s::any::Inner::I2s0(_) => 0x0,
                    esp_hal::i2s::any::Inner::I2s1(_) => 0xF,
                };

                // Route the I2S clock to the appropriate CLK_OUTn signal
                IO_MUX::regs().pin_ctrl().modify(|_, w| unsafe {
                    match clk_signal {
                        OutputSignal::CLK_OUT1 => w.clk1().bits(selector),
                        OutputSignal::CLK_OUT2 => w.clk2().bits(selector),
                        OutputSignal::CLK_OUT3 => w.clk3().bits(selector),
                        _ => unreachable!(),
                    }
                });

                clk_signal.connect_to(&mclk);
            }
        }
    }
}
