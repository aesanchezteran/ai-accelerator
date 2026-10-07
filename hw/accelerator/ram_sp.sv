/*
Code Developed by Gabriel Oña  
Dragon AI Accelerator - Proyecto Integrador  
Date: 07/04/2025 Quito - Ecuador

Module Details:
-------------------------------------------------------------------------------
Module Name      : ram_sp
Project          : Dragon AI Accelerator
Description      : 
  The `ram_sp` module implements a wrapper of a single-port BRAM template with configurable 
  width and depth. The read operation is combinational, while the write operation is sequential.
  Both operations require only a single cycle to complete. In the accelerator, this module is 
  used to store the activation scratchpad.
  
Inputs:
  - clk                     : Clock signal.
  - en                      : Enable signal to activate the memory operations.
  - we_a                    : Write enable signal for memory write operation.
  - addr_a                  : Address input for accessing memory.
  - data_in_a               : Data input for writing to memory.
  - re_a                    : Read enable signal for memory read operation.

Outputs:
  - data_out_a              : Data output for read operation from memory.
*/

module ram_sp #(
    parameter WIDTH = 16,
    parameter DEPTH = 4096
)(
    input logic                         clk,
    input logic                         en,
    
    input logic                         we_a,
    input logic [$clog2(DEPTH)-1:0]     addr_a,
    input logic [WIDTH-1:0]             data_in_a,
    input logic                         re_a,
    output logic [WIDTH-1:0]            data_out_a
    );
    
    logic [WIDTH-1:0] o_data;
    
    always_comb begin
        if(re_a & en) begin
            data_out_a = o_data;
        end else begin
            data_out_a = '0;
        end
    end
            
    rams_sp_rf_rst #(
        .WIDTH(WIDTH),
        .DEPTH(DEPTH)
    )ram_inst(
        .clk(clk),
        .en(en),
        .we(we_a),
        .addr(addr_a),
        .di(data_in_a),
        .dout(o_data)
    );
endmodule

