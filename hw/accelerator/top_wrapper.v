/* 
Code Developed by Gabriel Oña  
Dragon AI Accelerator - Proyecto Integrador  
Date: 07/04/2025  
Quito - Ecuador  

Module Details:
-------------------------------------------------------------------------------
Module Name      : top_dragon_ai_wrapper
Project          : Dragon AI Accelerator
Description      : 
  Verilog wrapper from top.sv to incorporate it as a custom IP with AXI interface.
  
Inputs:
  - clk                       : Clock signal.
  - rst                       : Reset signal.
  - instruction               : Instruction bus containing opcode and parameters.
  - dma_data_in               : Data input for the Direct Memory Access (DMA).
  - start                     : Signal to start the accelerator operations.

Outputs:
  - dma_address               : Output address for DMA.
  - dma_data_out              : Data output for DMA.
  - ready_interrupt           : Interrupt signal indicating the module is ready.
*/
module top_dragon_ai_wrapper#(
	// Parameters Override  
	parameter integer ARRAY_HEIGHT = 4,
	parameter integer ARRAY_WIDTH = 4,
	parameter integer PROCESSOR_BITWIDTH = 16,	
	//GENERAL PARAMETERS
    parameter SYSTOLIC_WORD = PROCESSOR_BITWIDTH,
    parameter OPCODE_SIZE = 4,
    //SYSTOLIC PARAMETERS
    parameter DATA_IN_SIZE = ARRAY_HEIGHT * SYSTOLIC_WORD,
    parameter DATA_OUT_SIZE = ARRAY_WIDTH * SYSTOLIC_WORD,
    //PS SCRATCHPAD PARAMETERS
    parameter PS_SP_DEPTH = 4096,
    parameter PS_SP_ADDR_SIZE = $clog2(PS_SP_DEPTH),
    parameter PS_SP_WIDTH = SYSTOLIC_WORD * ARRAY_WIDTH,
    //ACTIVATION SCRATCHPAD PARAMETERS
    parameter ACT_SP_DEPTH = 4096,
    parameter ACT_SP_ADDR_SIZE = $clog2(ACT_SP_DEPTH),
    parameter ACT_SP_WIDTH = SYSTOLIC_WORD * ARRAY_HEIGHT,
    //BUSES PARAMETERS
    parameter BUS_INSTRUCTION_SIZE = 64,
    parameter BUS_WEIGHT_SIZE = SYSTOLIC_WORD * ARRAY_WIDTH,
    parameter BUS_ACTIVATION_SIZE = SYSTOLIC_WORD * ARRAY_HEIGHT,
    parameter BUS_PS_SIZE = SYSTOLIC_WORD * ARRAY_WIDTH,
    //MULTICASTING PARAMETERS
    parameter ID_ROW_BITWIDTH = $clog2(ARRAY_HEIGHT),
    parameter BUS_MULTICAST_IN_SIZE = BUS_WEIGHT_SIZE + ID_ROW_BITWIDTH,
    //DMA ADDRESS
    parameter DMA_ADDRESS_SIZE = 32
)(
    input wire                                  clk,
    input wire                                  rst,
    input wire                                  axi_irq_clear,
    input wire                                  axi_irq_enable,
    input wire [BUS_INSTRUCTION_SIZE-1:0]       instruction,
    input wire [DATA_IN_SIZE-1:0]               dma_data_in,
    input wire                                  start,
    output wire [DMA_ADDRESS_SIZE-1:0]          dma_address,
    output wire [DATA_OUT_SIZE-1:0]             dma_data_out,
    output wire                                 interrupt
    );
	
    top #(
	   // Parameters Override  
		.ARRAY_HEIGHT(ARRAY_HEIGHT),
		.ARRAY_WIDTH(ARRAY_WIDTH),
		.PROCESSOR_BITWIDTH(PROCESSOR_BITWIDTH)
	) my_accelerator(
        .clk(clk),
        .rst(rst),
        .enable(start),
        .axi_irq_clear(axi_irq_clear),
        .axi_irq_enable(axi_irq_enable),
        .bus_instr(instruction),
        .dma_data_in(dma_data_in),
        .dma_address(dma_address),
        .dma_data_out(dma_data_out),
        .interrupt(interrupt)
    );
    
endmodule
