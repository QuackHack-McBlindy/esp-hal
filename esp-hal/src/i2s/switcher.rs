//! I2S half-duplex switcher for sharing clock pins between RX and TX.

use crate::{
    dma::DmaDescriptor,
    gpio::{OutputPin, interconnect::PeripheralInput},
    i2s::master::{I2s, I2sRx, I2sTx},
    Blocking,
};

/// A wrapper that holds an I2S instance and separate DMA descriptors for RX and TX.
pub struct I2sSwitcher<'d> {
    i2s: I2s<'d, Blocking>,
    rx_descriptors: &'static mut [DmaDescriptor],
    tx_descriptors: &'static mut [DmaDescriptor],
}

impl<'d> I2sSwitcher<'d> {
    /// Creates a new switcher.
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

    /// Convert into an I2S receiver (RX) with the given data input pin.
    pub fn into_rx(mut self, din: impl PeripheralInput<'d>) -> I2sRx<'d, Blocking> {
        self.i2s.init_shared_clocks();
        self.i2s.i2s_rx.with_din(din).build(self.rx_descriptors)
    }

    /// Convert into an I2S transmitter (TX) with the given data output pin.
    pub fn into_tx(mut self, dout: impl OutputPin + 'd) -> I2sTx<'d, Blocking> {
        self.i2s.init_shared_clocks();
        self.i2s.i2s_tx.with_dout(dout).build(self.tx_descriptors)
    }
}
