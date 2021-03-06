module core#(
  parameter PROGRAM_MEMORY_P = "",
  parameter DATA_WIDTH_P = 32,
  parameter DATA_ADDR_WIDTH_P = 32,
  parameter ADDR_WIDTH_P = 5,
  parameter CNTRL_WIDTH_P = 3,
  parameter ALU_CNTRL_WIDTH_P = 3,
  parameter FUNCT_WIDTH_P = 6,
  parameter OP_WIDTH_P = 6
)(
  input wire clk,
  input wire reset,
  input wire i_enable,

  // data memory interface
  input wire [DATA_WIDTH_P-1:0] i_mem_rd_data,
  output wire o_mem_wr_en,
  output wire [DATA_ADDR_WIDTH_P-1:0] o_mem_addr,
  output wire [DATA_WIDTH_P-1:0] o_mem_wr_data);

  //----------------------------------------------------------------------------
  // register and wire instantiations
  //----------------------------------------------------------------------------

  // note in MIPS programs are normally stored at starting address 0x00400000
  reg [DATA_WIDTH_P-1:0] pc = 32'h00000000;
  wire [DATA_WIDTH_P-1:0] pc_add;
  reg [DATA_WIDTH_P-1:0] pc_next = 32'h00000000;
  reg [DATA_WIDTH_P-1:0] sign_extend_imm;
  wire enable_pc;
  wire pc_wr_en;
  wire pc_next_sel;

  // instruction and data memory
  wire [DATA_WIDTH_P-1:0] instr;
  wire  [DATA_WIDTH_P-1:0] instr_d;
  wire instr_wr_en;
  wire instr_data_addr_sel;
  wire [DATA_WIDTH_P-1:0] instr_data_addr;
  wire instr_data_wr_en;
  wire [DATA_WIDTH_P-1:0] instr_data_wr_data;

  // ALU
  wire [ALU_CNTRL_WIDTH_P-1:0] alu_control;
  wire alu_in_a_sel;
  wire [DATA_WIDTH_P-1:0] alu_in_a;
  wire alu_in_b_sel;
  wire [DATA_WIDTH_P-1:0] alu_in_b;
  wire [DATA_WIDTH_P-1:0] alu_out;
  reg [DATA_WIDTH_P-1:0] alu_out_d;

  reg [DATA_WIDTH_P-1:0] mem_wr_data = {DATA_WIDTH_P[1'b0]};

  reg [31:0] instr_data_memory [0:255];
  reg enable_d = 1'b0;
  
  wire reg_wr_data_sel;
  wire mem_wr_en;
  wire branch;
  wire alu_src_sel;
  wire reg_wr_addr_sel;
  wire [DATA_WIDTH_P-1:0] reg_wr_data;
  wire [ADDR_WIDTH_P-1:0] reg_wr_addr;
  wire reg_wr_en;
  wire [DATA_WIDTH_P-1:0] reg_rd_port_a;
  wire [DATA_WIDTH_P-1:0] reg_rd_port_b;
  wire [DATA_WIDTH_P-1:0] beq_pc;
  wire [DATA_WIDTH_P-1:0] j_type_jump;
  wire jump;
  wire zero_alu_result;



  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // assignments
  //----------------------------------------------------------------------------

  // data memory interface assignments
  assign o_mem_wr_en  = mem_wr_en;
  assign o_mem_addr = alu_out;
  assign o_mem_wr_data = reg_rd_port_b;

  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // Control unit
  //----------------------------------------------------------------------------

  control_unit #(
    .ALU_CNTRL_WIDTH_P(ALU_CNTRL_WIDTH_P),
    .FUNCT_WIDTH_P(FUNCT_WIDTH_P),
    .OP_WIDTH_P(OP_WIDTH_P))
  cntrl_unit_i(
    .i_opcode(instr[DATA_WIDTH_P-1:26]),
    .i_function(instr[5:0]),
    .o_mem_wr_en(mem_wr_en),
    .o_branch(branch),
    .o_alu_cntrl(alu_control),
    .o_alu_src_sel(alu_src_sel),
    .o_reg_wr_addr_sel(reg_wr_addr_sel),
    .o_reg_wr_en(reg_wr_en),
    .o_reg_wr_data_sel(reg_wr_data_sel),
    .o_jump(jump));

  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // program counter
  //----------------------------------------------------------------------------

  // counter process
  always @(posedge clk) begin
    if (reset) begin
      pc <= 32'h00000000;
    end else if (enable_pc) begin
      pc <= pc_next;
    end;
  end
  
  assign enable_pc = (zero_alu_result & branch) | pc_wr_en;
  assign pc_next = pc_next_sel ? alu_out_d : alu_out;

  assign pc_add = p;

  // BEQ logic
  assign beq_pc = pc_add + (sign_extend_imm << 2);

  // JUMP logic
  assign j_type_jump = {pc_add[DATA_WIDTH_P-1:28],instr[25:0],2'b00};

  // next pc multiplexer
  always @(*) begin
    case({jump,branch & zero_alu_result})
      2'b00 : pc_next = pc_add;
      2'b01 : pc_next = beq_pc;
      2'b10 : pc_next = j_type_jump;
      default : pc_next = pc_add;
    endcase
  end

  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // Instruction and data memory
  //----------------------------------------------------------------------------

  // program memory (combinatorial reads), 32 bit word addressable only
  initial begin
    if (PROGRAM_MEMORY_P != "") begin
      $readmemh(PROGRAM_MEMORY_P, instr_data_memory);
    end
  end

  // address select between ALU output or program counter
  assign instr_data_addr = instr_data_addr_sel ? alu_out_d : pc;

  // read process
  always @(posedge clk) begin
    instr_data_out <= instr_data_memory[instr_data_addr];
  end

  // write process
  assign instr_data_wr_data = reg_rd_port_b;
  always @(posedge clk) begin
    if (instr_data_wr_en == 1'b1) begin
      instr_data_memory[instr_data_wr_addr] <= instr_data_wr_data;
    end
  end

  assign instr_d = instr_wr_en ? instr_data_out : {DATA_WIDTH_P{1'b0}};

  // sign extension for LW
  assign sign_extend_imm = {{16{instr_d[15]}},instr_d[15:0]};

  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // register file
  //----------------------------------------------------------------------------

  register_file #(
    .DATA_WIDTH_P(DATA_WIDTH_P),
    .ADDR_WIDTH_P(ADDR_WIDTH_P))
  reg_i (
    .clk(clk),
    .reset(reset),
    .i_rd_addr_a(instr_d[25:21]),
    .i_rd_addr_b(instr_d[20:16]),
    .i_wr_addr(reg_wr_addr),
    .i_wr_data(instr_data_out),
    .i_wr_enable(reg_wr_en),
    .o_rd_data_a(reg_rd_port_a),
    .o_rd_data_b(reg_rd_port_b));

  // write data select
  assign reg_wr_data = reg_wr_data_sel ? instr_data_out : alu_out_d;

  // write address select
  assign reg_wr_addr = reg_wr_addr_sel ? instr[15:11] : instr[20:16];

  //----------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // ALU
  //----------------------------------------------------------------------------

  alu #(
    .DATA_WIDTH_P(DATA_WIDTH_P),
    .ADDR_WIDTH_P(ADDR_WIDTH_P),
    .CNTRL_WIDTH_P(ALU_CNTRL_WIDTH_P))
  alu_i(
    .clk(clk),
    .reset(reset),
    .i_control(alu_control),
    .i_a(alu_in_a),
    .i_b(alu_in_b),
    .o_result(alu_out));

  // ALU source select A
  assign alu_in_a = alu_in_a_sel ? reg_rd_port_a : pc;

  // ALU source select B
  always @(*) begin
    case(alu_in_b)
      00      : alu_in_b = reg_rd_port_b;
      01      : alu_in_b = 4;
      10      : alu_in_b = sign_extend_imm;
      11      : alu_in_b = sign_extend_imm << 2;
      default : alu_in_b = {DATA_WIDTH_P{1'bx}};
    endcase
  end

  always @(posedge clk) begin
    alu_out_d <= alu_out;
  end

  // src b select
  assign alu_in_b = alu_src_sel ? sign_extend_imm : reg_rd_port_b;

  // zero detect
  assign zero_alu_result = alu_out == {DATA_ADDR_WIDTH_P[1'b0]} ? 1'b1 : 1'b0;

  //----------------------------------------------------------------------------

endmodule