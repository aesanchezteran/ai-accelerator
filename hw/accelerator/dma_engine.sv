/* 
Code Developed by Gabriel Oña  
Dragon AI Accelerator - Proyecto Integrador  
Date: 07/04/2025  Quito - Ecuador  

Module Details:
-------------------------------------------------------------------------------
Module Name      : dma_engine
Project          : Dragon AI Accelerator
Description      : 
  The dma_engine module is responsible for managing Direct Memory Access (DMA) 
  operations within the Dragon AI Accelerator and the DDR Memory. It decodes 
  opcodes to perform various tasks such as storing weights, activations, 
  partial sums, and collecting partial sums. The module handles data transfers 
  between different memory regions, including activation and partial sum scratchpads.
  In general, this module manages data transactions between off-chip memory with 
  internal scratchpads.

Inputs:
  - enable                  : Enables the module's operation.
  - deco_address            : The address for the DMA operation.
  - opcode                  : The opcode specifying the type of operation.
  - dma_data_in             : Input data to be transferred.
  - ps_collect_data         : Data to be collected from the partial sum memory.
    
Outputs:
  - dma_address             : The address for the DMA operation.
  - act_data                : Output activation data.
  - ps_data                 : Output partial sum data.
  - weight_data             : Output weight data.
  - dma_data_out            : Output data from the DMA operation.
*/
`include "enums.sv"

module dma_engine#(
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
    input logic                             enable,
    input logic [DMA_ADDRESS_SIZE-1:0]      deco_address,
    input logic [OPCODE_SIZE-1:0]           opcode,
    input logic [DATA_IN_SIZE-1:0]          dma_data_in,
    input logic [DATA_OUT_SIZE-1:0]         ps_collect_data,
    
    output logic [DMA_ADDRESS_SIZE-1:0]     dma_address,
    output logic [ACT_SP_WIDTH-1:0]         act_data,
    output logic [PS_SP_WIDTH-1:0]          ps_data,
    output logic [BUS_WEIGHT_SIZE-1:0]      weight_data,
    output logic [DATA_OUT_SIZE-1:0]        dma_data_out
    );
    
    //OPCODE CASTING
    opcode_enum_t opcode_enum;
    always_comb begin
        opcode_enum = opcode_enum_t'(opcode);
    end
    
    always_comb begin
        if (enable) begin
            case(opcode_enum)
                IDLE_op: begin
                    dma_address = '0;
                    act_data = '0;
                    ps_data = '0;
                    weight_data = '0;
                    dma_data_out = '0;
                end
                WEIGHT_STORE_op: begin
                    dma_address = deco_address;
                    act_data = '0;
                    ps_data = '0;
                    weight_data = dma_data_in;
                    dma_data_out = '0;
                end
                ACT_STORE_op: begin
                    dma_address = deco_address;
                    act_data = dma_data_in;
                    ps_data = '0;
                    weight_data = '0;
                    dma_data_out = '0;
                end
                PSUM_STORE_op: begin
                    dma_address = deco_address;
                    act_data = '0;
                    ps_data = dma_data_in;
                    weight_data = '0;
                    dma_data_out = '0;
                end
                MM_op: begin
                    dma_address = '0;
                    act_data = '0;
                    ps_data = '0;
                    weight_data = '0;
                    dma_data_out = '0;
                end
                PSUM_ACC_op: begin
                    dma_address = '0;
                    act_data = '0;
                    ps_data = '0;
                    weight_data = '0;
                    dma_data_out = '0;
                end
                PS_COLL_op: begin
                    dma_address = deco_address;
                    act_data = '0;
                    ps_data = '0;
                    weight_data = '0;
                    dma_data_out = ps_collect_data;
                end
                default: begin
                    dma_address = '0;
                    act_data = '0;
                    ps_data = '0;
                    weight_data = '0;
                    dma_data_out = '0;
                end
            endcase
         end else begin
            dma_address = '0;
            act_data = '0;
            ps_data = '0;
            weight_data = '0;
            dma_data_out = '0;
         end
    end
endmodule
