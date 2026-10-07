/*
Code Developed by Gabriel Oña  
Dragon AI Accelerator - Proyecto Integrador  
Date: 07/04/2025 Quito - Ecuador

Module Details:
-------------------------------------------------------------------------------
Module Name      : ram_dp
Project          : Dragon AI Accelerator
Description      : 
  The `ram_dp` module implements a wrapper of a dual-port BRAM template with configurable 
  width and depth. The read operation is sequential, as well as the write operation.
  Therefore, the read operation requires an additional clock cycle to execute compared with 
  single-port BRAM. In the accelerator, this module is used to store the partial sum scratchpad.

Inputs:
  - clk                     : Clock signal.
  - en                      : Enable signal for the module operation.
  - we_a                    : Write enable signal for port A.
  - addr_a                  : Address input for port A.
  - data_in_a               : Data input for port A.
  - re_a                    : Read enable signal for port A.
  - we_b                    : Write enable signal for port B.
  - addr_b                  : Address input for port B.
  - data_in_b               : Data input for port B.
  - re_b                    : Read enable signal for port B.

Outputs:
  - data_out_a              : Data output from port A.
  - data_out_b              : Data output from port B.
*/

module ram_dp #(
    parameter       WIDTH   = 16,
    parameter       DEPTH   = 4096
)(
    input   logic                           clk,
    input   logic                           en,
    
    input   logic                           we_a,
    input   logic   [$clog2(DEPTH)-1:0]     addr_a,
    input   logic   [WIDTH-1:0]             data_in_a,
    input   logic                           re_a,
    output  logic   [WIDTH-1:0]             data_out_a,
    
    input   logic                           we_b,
    input   logic   [$clog2(DEPTH)-1:0]     addr_b,
    input   logic   [WIDTH-1:0]             data_in_b,
    input   logic                           re_b,
    output  logic   [WIDTH-1:0]             data_out_b
    );
        
    logic [WIDTH-1:0] o_data_a;
    logic [WIDTH-1:0] o_data_b;
            
    rams_tdp_rf_rf #(
        .WIDTH(WIDTH),
        .DEPTH(DEPTH)
    )ram_inst(
        .clka(clk),
        .clkb(clk),
        .ena(en),
        .enb(en),
        .wea(we_a),
        .web(we_b),
        .addra(addr_a),
        .addrb(addr_b),
        .dia(data_in_a),
        .dib(data_in_b),
        .doa(o_data_a),
        .dob(o_data_b)
    );
    always_comb begin
        if(re_a & en) begin
            data_out_a = o_data_a;
        end else begin
            data_out_a = '0;
        end
        if(re_b & en) begin
            data_out_b = o_data_b;
        end else begin
            data_out_b = '0;
        end
    end
endmodule
