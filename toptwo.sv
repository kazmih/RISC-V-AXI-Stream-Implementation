module Control_Unit(opcode , Branch, Jump , ImmSrc , ResultSrc , ALUOp , MemWrite , ALUSrc , RegWrite);
input [6:0] opcode;
output reg Branch , Jump , MemWrite , ALUSrc , RegWrite;
output reg [1:0] ImmSrc , ALUOp ,  ResultSrc;
always @(*) begin
case (opcode)
7'b0110011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b0_00_xx_1_0_0_10_0;
7'b0000011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_01_00_1_0_0_00_0;
7'b0100011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_xx_01_0_1_0_00_0;
7'b1100011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b0_xx_10_0_0_1_01_0;
7'b0010011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_00_00_1_0_0_10_0;
7'b1101111 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'bx_10_11_1_0_0_xx_1;
default    : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b0_00_00_0_0_0_00_0;
endcase
end
endmodule
module ALU_Control(ALUOp , Funct3 , Funct7 , op , Operation);
input [1:0] ALUOp;
input [2:0] Funct3;
input Funct7 , op;
output reg [2:0] Operation;

always @(*) begin
case(ALUOp)
2'b00: Operation = 3'b000;  // add for load/store
2'b01: Operation = 3'b001;  // sub for branch
2'b10: begin               // R-type/I-type
case(Funct3)
3'b000: Operation = (op && Funct7) ? 3'b001 : 3'b000; // sub if R-type and Funct7=1, add otherwise
3'b010: Operation = 3'b101; // slt
3'b110: Operation = 3'b011; // or
3'b111: Operation = 3'b010; // and
default: Operation = 3'b000;
endcase
end
default: Operation = 3'b000;
endcase
end
endmodule
module CU(Zero , opcode , Funct3 , Funct7 , PCSrc , ResultSrc , MemWrite , ALUSrc , ImmSrc , RegWrite , Operation);
input Zero;
input [6:0] opcode ,Funct7;
input [2:0] Funct3;
output  PCSrc , MemWrite , ALUSrc , RegWrite;
output  [1:0] ImmSrc , ResultSrc;
output  [2:0] Operation;
wire Branch , Jump;
wire [1:0] ALUOp;
Control_Unit c(opcode , Branch , Jump , ImmSrc , ResultSrc , ALUOp , MemWrite , ALUSrc , RegWrite);
ALU_Control  a(ALUOp , Funct3 , Funct7[5] , opcode[5] , Operation);
assign PCSrc = Jump | (Zero & Branch);
endmodule



module registerFile (
input wire clk,             // Clock signal
input wire RegWrite,        // Write enable signal
input wire [4:0] RS1,       // Source register 1 (5 bits, selects one of 32 registers)
input wire [4:0] RS2,       // Source register 2 (5 bits, selects another of 32 registers)
input wire [4:0] RD,        // Destination register (5 bits, selects the register to write to)
input wire [31:0] WriteData, // Data to be written to register RD (32 bits wide)
output wire [31:0] ReadData1, // Data read from register RS1 (32 bits wide)
output wire [31:0] ReadData2  // Data read from register RS2 (32 bits wide)
);

// 32 registers, each 32 bits wide
reg [31:0] Registers [31:0];

// Initialize registers with random values
integer i;
initial begin
for (i = 0; i < 32; i = i + 1)
Registers[i] = i;  // Initialize with random values for simulation
end

// Read operation (asynchronous)
assign ReadData1 = Registers[RS1];  // Read register RS1
assign ReadData2 = Registers[RS2];  // Read register RS2

// Write operation (synchronous, on positive clock edge)
always @(posedge clk) begin
    if (RegWrite && (RD != 5'b00000)) begin
        Registers[RD] <= WriteData;
    end
end


endmodule



module alu(a, b, op, res, zero);
input [31:0] a, b;
input [2:0] op;
output reg zero;
output reg [31:0] res;

always @(*) begin
case(op)
3'b000: res = a + b;    // ADD
3'b001: res = a - b;    // SUB
3'b101: res = a < b;    // SLT
3'b011: res = a | b;    // OR
3'b010: res = a & b;    // AND
default: res = 32'b0;
endcase

zero = (res == 32'b0) ? 1'b1 : 1'b0;
end
endmodule



module imm_data_gen(instruction , ImmSrc , imm_data);
input [31:0] instruction;
input [1:0] ImmSrc;
output reg [31:0] imm_data;
always @(*) begin
case(ImmSrc)
2'b00: imm_data = {{20{instruction[31]}}, instruction[31:20]};
2'b01: imm_data = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
2'b10: imm_data = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
2'b11: imm_data = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
default: imm_data = 32'b0;
endcase
end
endmodule


module Data_Memory(
input wire [31:0] Mem_Addr,
input wire [31:0] Write_Data,
input wire clk,
input wire MemWrite,
output wire [31:0] Read_Data
);

reg [7:0] memory [63:0];

integer i;
initial begin
for (i = 0; i < 64; i = i + 1)
memory[i] = i;
end

assign Read_Data = {memory[Mem_Addr + 3], memory[Mem_Addr + 2], memory[Mem_Addr + 1], memory[Mem_Addr]};

always @(posedge clk) begin
if (MemWrite) begin
{memory[Mem_Addr + 3], memory[Mem_Addr + 2], memory[Mem_Addr + 1], memory[Mem_Addr]} = Write_Data;
end
end
endmodule




module Program_Counter (clk , rst , PC_In , PC_Out);
input clk , rst;
input [31:0] PC_In;
output reg [31:0] PC_Out;
always @(posedge clk) begin
if (rst)
PC_Out <= 0;
else
PC_Out <= PC_In;
end
endmodule


module adder (a , b , out);
input [31:0] a , b;
output [31:0] out;
assign out = a + b ;
endmodule


module Instruction_Memory(Inst_Address , Instruction);
input wire [31:0] Inst_Address;
output reg [31:0] Instruction;
reg [7:0] memory [31:0];
initial begin
// 0x00: lui x1, 0xFFFFF     # Load upper bits: x1 = 0xFFFFF000
memory[0] = 8'h37;
memory[1] = 8'hF0;
memory[2] = 8'hFF;
memory[3] = 8'hFF;

// 0x04: addi x1, x1, 0x800  # Add lower bits: x1 = 0xFFFFF800
memory[4] = 8'h13;
memory[5] = 8'h85;
memory[6] = 8'h00;
memory[7] = 8'h80;

// 0x08: addi x2, x0, 5      # Load test data: x2 = 5
memory[8] = 8'h13;
memory[9] = 8'h01;
memory[10] = 8'h50;
memory[11] = 8'h00;

// 0x0C: nop                 # No operation (addi x0, x0, 0)
memory[12] = 8'h13;
memory[13] = 8'h00;
memory[14] = 8'h00;
memory[15] = 8'h00;

// 0x10: nop                 # No operation (addi x0, x0, 0)
memory[16] = 8'h13;
memory[17] = 8'h00;
memory[18] = 8'h00;
memory[19] = 8'h00;

// 0x14: sw x2, 0(x1)        # Store x2 to address in x1 (0xFFFFF800)
memory[20] = 8'h23;
memory[21] = 8'h20;
memory[22] = 8'h25;
memory[23] = 8'h00;

// 0x18: jal x0, -24         # Jump back to start
memory[24] = 8'h6F;
memory[25] = 8'hF0;
memory[26] = 8'h5F;
memory[27] = 8'hFF;

// Unused but initialized to avoid x's in simulation
memory[28] = 8'h00;
memory[29] = 8'h00;
memory[30] = 8'h00;
memory[31] = 8'h00;
end


always @(*) begin
Instruction = {memory[Inst_Address + 3], memory[Inst_Address + 2], memory[Inst_Address + 1], memory[Inst_Address]};
end
endmodule


module mux32bit(a,b,c,sel,out);
input [31:0] a,b,c;
input [1:0] sel;
output reg [31:0] out;
always @(*) begin
case(sel)
2'b00 : out <= a;
2'b01 : out <= b;
2'b10 : out <= c;
default: out <=32'b0;
endcase
end
endmodule


module mux32bit2(a,b,sel,out);
input [31:0] a,b;
input  sel;
output reg [31:0] out;
always @(*) begin
case(sel)
2'b0 : out <= a;
2'b1 : out <= b;
default: out <=32'b0;
endcase
end
endmodule


module Address_Decoder (Mem_Addr, MemWrite, newd);
input [31:0] Mem_Addr;
input MemWrite;
output reg newd;
parameter AXI_BASE_ADDR = 32'hFFFFF800;  // Match sign-extended value
parameter AXI_ADDR_MASK = 32'hFFFFF800; 
always @(*) begin
if (MemWrite && ((Mem_Addr & AXI_ADDR_MASK) == AXI_BASE_ADDR)) begin
newd = 1'b1; 
end
else begin
newd = 1'b0;
end
end
endmodule



module RISC_V_Processor(clk , rst , res , MemWrite , ReadData2);
input clk , rst;
output MemWrite;
output [31:0] res , ReadData2;
wire [31:0] PCPlus4 , PCin , PCTarget , PC_Out , Instruction , Result , ReadData1 , ReadData2 , res , Read_Data , imm_data , SrcB;
wire [2:0] Operation;
wire [1:0]  ImmSrc , ResultSrc;
wire RegWrite , PCSrc , ALUSrc , Zero , MemWrite;
adder pcp4(PC_Out , 32'b100 , PCPlus4);
mux32bit2 PCSrcMUX(PCPlus4 , PCTarget , PCSrc , PCin);
Program_Counter PC(clk , rst , PCin , PC_Out);
Instruction_Memory IM(PC_Out , Instruction);
mux32bit ResultSrcMUX(res , Read_Data ,PCPlus4 , ResultSrc , Result);
registerFile RF(clk , RegWrite , Instruction[19:15] , Instruction[24:20] , Instruction[11:7] , Result , ReadData1 , ReadData2);
imm_data_gen ID(Instruction , ImmSrc , imm_data);
adder pct(PC_Out , imm_data , PCTarget);
mux32bit2 ALUSrcMUX(ReadData2 , imm_data , ALUSrc , SrcB);
alu lua(ReadData1 , SrcB , Operation , res , Zero);
Data_Memory DM(res, ReadData2, clk, MemWrite, Read_Data);
CU controlunit(Zero , Instruction[6:0] , Instruction[14:12] , Instruction[31:25] , PCSrc , ResultSrc , MemWrite , ALUSrc , ImmSrc , RegWrite , Operation);
endmodule

module axis_m(
input  wire m_axis_aclk,
input  wire m_axis_aresetn,
input  wire newd,
input  wire [7:0] din,
input  wire m_axis_tready,
output wire m_axis_tvalid,
output wire [7:0] m_axis_tdata,
output wire m_axis_tlast
);
typedef enum bit {idle = 1'b0, tx = 1'b1} state_type;
state_type state = idle, next_state = idle;
reg [2:0] count = 0;
always@(posedge m_axis_aclk)
begin
if(m_axis_aresetn == 1'b0)
state <= idle;
else
state <= next_state;
end
always@(posedge m_axis_aclk)
begin
if(state == idle)
count <= 0;
else if(state == tx && count != 3 && m_axis_tready == 1'b1)
count <= count +1;
else
count <= count;
end
always@(*)
begin
case(state)
idle:
begin
if(newd == 1'b1)
next_state = tx;
else
next_state = idle;
end
tx:
begin
if(m_axis_tready == 1'b1)
begin
if(count != 3)
next_state  = tx;
else
next_state  = idle;
end
else
begin
next_state  = tx;
end
end
default: next_state = idle;
endcase
end
assign m_axis_tdata   = (m_axis_tvalid) ? din*count : 0;
assign m_axis_tlast   = (count == 3 && state == tx)    ? 1'b1 : 0;
assign m_axis_tvalid  = (state == tx ) ? 1'b1 : 1'b0;
endmodule

module FIFO_32_to_8 (
input wire clk,
input wire rst,
input wire [31:0] din,       
input wire write_enable,     
input wire read_enable,      
output reg [7:0] dout,       
output reg empty,            
output reg full              
);
// Internal storage for FIFO
reg [7:0] fifo_mem [0:3];    // Storage for 4x8-bit chunks
reg [1:0] write_ptr = 0;     // Write pointer
reg [1:0] read_ptr = 0;      // Read pointer
reg [1:0] chunk_counter = 0; // Counter for splitting 32-bit data
// FIFO logic
always @(posedge clk or posedge rst) begin
if (rst) begin
write_ptr <= 0;
read_ptr <= 0;
chunk_counter <= 0;
empty <= 1'b1;
full <= 1'b0;
end else begin
// Write operation
if (write_enable && !full) begin
fifo_mem[write_ptr] <= din[8*chunk_counter +: 8]; // Split 32-bit into 8-bit chunks
chunk_counter <= chunk_counter + 1;
if (chunk_counter == 3) begin
write_ptr <= write_ptr + 1;
chunk_counter <= 0;
end
empty <= 1'b0;
if (write_ptr == (read_ptr - 1)) full <= 1'b1;
end
// Read operation
if (read_enable && !empty) begin
dout <= fifo_mem[read_ptr];
read_ptr <= read_ptr + 1;
full <= 1'b0;
if (read_ptr == write_ptr) empty <= 1'b1;
end
end
end
endmodule

module axis_s(
input  wire s_axis_aclk,
input  wire s_axis_aresetn,
output wire s_axis_tready,
input  wire s_axis_tvalid,
input  wire [7:0] s_axis_tdata,
input  wire s_axis_tlast,
output wire [7:0] s_dout
);
typedef enum bit [1:0] {idle = 2'b00, store = 2'b01, last_byte = 2'b10} state_type;
state_type state = idle, next_state = idle;
always@(posedge s_axis_aclk)
begin
if(s_axis_aresetn == 1'b0)
state  <= idle;
else
state <= next_state;
end
always@(*)
begin
case(state)
idle:
begin
if(s_axis_tvalid == 1'b1)
next_state = store;
else
next_state = idle;
end
store:
begin
if(s_axis_tlast == 1'b1 && s_axis_tvalid == 1'b1 )
next_state = idle;
else if (s_axis_tlast == 1'b0 && s_axis_tvalid == 1'b1)
next_state = store;
else
next_state = idle;
end
default: next_state = idle;
endcase
end
assign s_axis_tready = (state == store);
assign dout          = (state == store ) ? s_axis_tdata : 8'h00;
endmodule



module toptwo(clk , rst);
    input clk , rst;
    wire MemWrite, newd;
    wire s_axis_tready, m_axis_tvalid, m_axis_tlast;
    wire empty, full;
    wire [7:0] m_axis_tdata, s_dout, dout;
    wire [31:0] res, ReadData2;
    
    // Add these missing wire declarations
    wire write_enable, read_enable; 
    
    RISC_V_Processor RV(clk, rst, res, MemWrite, ReadData2);
    Address_Decoder AD(res, MemWrite, newd);
    
    // Fix the axis_m instantiation - add missing din parameter
    axis_m am(
        .m_axis_aclk(clk),
        .m_axis_aresetn(~rst),
        .newd(newd),
        .din(dout),                 // Connect to FIFO output
        .m_axis_tready(s_axis_tready),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tlast(m_axis_tlast)
    );
    
    axis_s as(
        .s_axis_aclk(clk),
        .s_axis_aresetn(~rst),
        .s_axis_tready(s_axis_tready),
        .s_axis_tvalid(m_axis_tvalid),
        .s_axis_tdata(m_axis_tdata),
        .s_axis_tlast(m_axis_tlast),
        .s_dout(s_dout)
    );
    
    FIFO_32_to_8 fifo(
        .clk(clk),
        .rst(rst),
        .din(ReadData2),
        .write_enable(write_enable),
        .read_enable(read_enable),
        .dout(dout),
        .empty(empty),
        .full(full)
    );
    
    // Fix these assignments
    assign write_enable = newd & ~full;  // Allow writes only if not full
    assign read_enable = ~empty & s_axis_tready; // Allow reads only if not empty and AXI is ready
endmodule




module toptwo_tb();
    // Clock and Reset
    logic clk, rst;
    
    // Instantiate the RISC-V Processor with AXI
    toptwo uut(clk, rst);
    
    ///////////////////////////////////////
    // Signal monitoring for all modules //
    ///////////////////////////////////////
    
    // === RISC-V Processor Signals ===
    // Program Counter signals
    wire [31:0] PC_Out, PCPlus4, PCTarget, PCin;
    // Instruction and decoding signals
    wire [31:0] Instruction;
    // Register file signals
    wire [31:0] ReadData1, ReadData2, Result;
    // ALU signals
    wire [31:0] res, SrcB;
    wire [2:0] Operation;
    wire Zero;
    // Data Memory signals
    wire [31:0] Read_Data;
    // Immediate Generator
    wire [31:0] imm_data;
    // Control signals
    wire [1:0] ImmSrc, ResultSrc;
    wire RegWrite, PCSrc, ALUSrc, MemWrite;
    wire [1:0] ALUOp;
    wire Branch, Jump;
    
    // === Address Decoder Signals ===
    wire newd;
    
    // === FIFO Signals ===
    wire write_enable, read_enable;
    wire empty, full;
    wire [7:0] dout;
    
    // === AXI Stream Master Signals ===
    wire m_axis_tvalid, m_axis_tlast;
    wire [7:0] m_axis_tdata;
    wire [2:0] count;  // Counter in the state machine
    wire state_m, next_state_m;  // State machine states
    
    // === AXI Stream Slave Signals ===
    wire s_axis_tready;
    wire [7:0] s_dout;
    wire [1:0] state_s, next_state_s;  // State machine states
    
    // === Assignments for RISC-V Processor ===
    assign PC_Out = uut.RV.PC_Out;
    assign PCPlus4 = uut.RV.PCPlus4;
    assign PCTarget = uut.RV.PCTarget;
    assign PCin = uut.RV.PCin;
    assign Instruction = uut.RV.Instruction;
    assign Result = uut.RV.Result;
    assign ReadData1 = uut.RV.ReadData1;
    assign ReadData2 = uut.RV.ReadData2;
    assign res = uut.RV.res;
    assign Read_Data = uut.RV.Read_Data;
    assign imm_data = uut.RV.imm_data;
    assign SrcB = uut.RV.SrcB;
    assign Operation = uut.RV.Operation;
    assign ImmSrc = uut.RV.ImmSrc;
    assign ResultSrc = uut.RV.ResultSrc;
    assign RegWrite = uut.RV.RegWrite;
    assign PCSrc = uut.RV.PCSrc;
    assign ALUSrc = uut.RV.ALUSrc;
    assign Zero = uut.RV.Zero;
    assign MemWrite = uut.RV.MemWrite;
    assign ALUOp = uut.RV.controlunit.ALUOp;
    assign Branch = uut.RV.controlunit.Branch;
    assign Jump = uut.RV.controlunit.Jump;
    
    // === Assignments for Address Decoder ===
    assign newd = uut.newd;
    
    // === Assignments for FIFO ===
    assign write_enable = uut.write_enable;
    assign read_enable = uut.read_enable;
    assign empty = uut.empty;
    assign full = uut.full;
    assign dout = uut.dout;
    
    // === Assignments for AXI Stream Master ===
    assign m_axis_tvalid = uut.m_axis_tvalid;
    assign m_axis_tdata = uut.m_axis_tdata;
    assign m_axis_tlast = uut.m_axis_tlast;
    assign count = uut.am.count;
    assign state_m = uut.am.state;
    assign next_state_m = uut.am.next_state;
    
    // === Assignments for AXI Stream Slave ===
    assign s_axis_tready = uut.s_axis_tready;
    assign s_dout = uut.s_dout;
    assign state_s = uut.as.state;
    assign next_state_s = uut.as.next_state;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Test sequence
    initial begin
        // Apply reset
        rst = 1;
        #15 rst = 0;
        
        // Run simulation for a sufficient time
        #500;
        
        // End simulation
        $finish;
    end
    
    // Monitor signals
    initial begin
        $display("Starting simulation...");
        
        // Create a more organized monitoring system by category
        $display("\n=== RISC-V Processor Signals ===");
        $monitor("Time=%0t | PC=%h | Instr=%h | ReadData1=%h | ReadData2=%h | imm=%h | ALUres=%h | RegWrite=%b",
                 $time, PC_Out, Instruction, ReadData1, ReadData2, imm_data, res, RegWrite);
            
        fork
            // Monitor Address Decoder
            begin
                forever begin
                    @(posedge clk);
                    $display("\n=== Address Decoder ===");
                    $display("Time=%0t | MemWrite=%b | res=%h | newd=%b", 
                             $time, MemWrite, res, newd);
                end
            end
            
            // Monitor FIFO
            begin
                forever begin
                    @(posedge clk);
                    $display("\n=== FIFO Signals ===");
                    $display("Time=%0t | write_enable=%b | read_enable=%b | empty=%b | full=%b | dout=%h", 
                             $time, write_enable, read_enable, empty, full, dout);
                end
            end
            
            // Monitor AXI Master
            begin
                forever begin
                    @(posedge clk);
                    $display("\n=== AXI Master Signals ===");
                    $display("Time=%0t | state=%b | next_state=%b | count=%d | tvalid=%b | tdata=%h | tlast=%b", 
                             $time, state_m, next_state_m, count, m_axis_tvalid, m_axis_tdata, m_axis_tlast);
                end
            end
            
            // Monitor AXI Slave
            begin
                forever begin
                    @(posedge clk);
                    $display("\n=== AXI Slave Signals ===");
                    $display("Time=%0t | state=%b | next_state=%b | tready=%b | s_dout=%h", 
                             $time, state_s, next_state_s, s_axis_tready, s_dout);
                end
            end
        join_none
    end
    
endmodule
