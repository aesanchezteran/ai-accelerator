`ifndef CONFIG_VH
`define CONFIG_VH

/*
This file is dedicated to store all the parameters to build the 
USFQ - Accelerator.
Written by Gabriel Ona
*/

    //GENERAL PARAMETERS
    parameter SYSTOLIC_WORD = 16;
    parameter OPCODE_SIZE = 4;
    
    //SYSTOLIC PARAMETERS
    parameter ARRAY_HEIGHT = 4;
    parameter ARRAY_WIDTH = 4;
    parameter DATA_IN_SIZE = ARRAY_HEIGHT * SYSTOLIC_WORD;
    parameter DATA_OUT_SIZE = ARRAY_WIDTH * SYSTOLIC_WORD;
    
    //PS SCRATCHPAD PARAMETERS
    parameter PS_SP_DEPTH = 4096;
    parameter PS_SP_ADDR_SIZE = $clog2(PS_SP_DEPTH);
    parameter PS_SP_WIDTH = SYSTOLIC_WORD * ARRAY_WIDTH;
    
    //ACTIVATION SCRATCHPAD PARAMETERS
    parameter ACT_SP_DEPTH = 4096;
    parameter ACT_SP_ADDR_SIZE = $clog2(ACT_SP_DEPTH);
    parameter ACT_SP_WIDTH = SYSTOLIC_WORD * ARRAY_HEIGHT;

    //BUSES PARAMETERS
    parameter BUS_INSTRUCTION_SIZE = 64;
    parameter BUS_WEIGHT_SIZE = SYSTOLIC_WORD * ARRAY_WIDTH;
    parameter BUS_ACTIVATION_SIZE = SYSTOLIC_WORD * ARRAY_HEIGHT;
    parameter BUS_PS_SIZE = SYSTOLIC_WORD * ARRAY_WIDTH;

    //MULTICASTING PARAMETERS
    parameter ID_ROW_BITWIDTH = $clog2(ARRAY_HEIGHT); 
    parameter BUS_MULTICAST_IN_SIZE = BUS_WEIGHT_SIZE + ID_ROW_BITWIDTH;

    //DMA ADDRESS
    parameter DMA_ADDRESS_SIZE = 32;
`endif