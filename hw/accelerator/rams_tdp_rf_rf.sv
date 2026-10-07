
module rams_tdp_rf_rf #(
    parameter       WIDTH   = 16,
    parameter       DEPTH   = 4096
)(
    input logic clka,
    input logic clkb,
    input logic ena,
    input logic enb,
    input logic wea,
    input logic web,
    
    input logic [$clog2(DEPTH)-1:0] addra,
    input logic [$clog2(DEPTH)-1:0] addrb,
    input logic [WIDTH-1:0] dia,
    input logic [WIDTH-1:0] dib,
    output logic [WIDTH-1:0] doa,
    output logic [WIDTH-1:0] dob
    );
    
    logic [WIDTH-1:0] ram [DEPTH-1:0];
    
    always_ff @(posedge clka) begin
        if (ena) begin
            if (wea) begin
                ram[addra] <= dia;
            end
            doa <= ram[addra];
        end
    end 
    
    always_ff @(posedge clkb) begin
        if (enb) begin
            if (web) begin
                ram[addrb] <= dib;
            end
            dob <= ram[addrb];
        end
    end 
endmodule
