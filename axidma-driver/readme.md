Current problem: 
Can't access any DMA Controller registers other than MM2S CTRL (offset 0x0) and S2MM CTRL (offset 0x30); reads always return 0x0 and writes have no effect.
The desing_1_wrapper.bin bitstream is a simple inverter + dual channel dma ip in order to test both dma channels (MM2S + S2MM).
