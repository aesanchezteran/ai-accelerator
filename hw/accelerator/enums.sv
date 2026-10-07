`ifndef ENUMS
`define ENUMS

/*
This file is dedicated to declare all enum variables in the project.
Written by Gabriel Ona
*/
    //PE Selector Enum
    typedef enum logic [1:0]{
        PE_IDLE,
        PE_WEIGHT_FETCH,
        PE_MULTIPLY
    }PE_selector_t;

    //MUX OUT SYSTOLIC ARRAY TO PS SCRATCHPAD
    typedef enum logic {
        MUX_SYSTOLIC,
        MUX_ALUS
    }MUX_PS_selector_t;
    
    //OPCODE ENUM
    typedef enum logic [3:0]{
        IDLE_op,
        WEIGHT_STORE_op,
        ACT_STORE_op,
        PSUM_STORE_op,
        MM_op,
        PSUM_ACC_op,
        PS_COLL_op
    }opcode_enum_t;
`endif
