package ISA

import chisel3.util.BitPat

object RV64Zfh extends Decoder32Bits {
  // RV32Zfh Standard Extension
  // I-type         |-----imm-----|-rs1-|-funct3-|--rd-|opcode|
  def flh = BitPat("b???????_?????_?????___001____?????_00001")

  // S-type         |--imm--|-rs2-|-rs1-|-funct3-|-imm-|opcode|
  def fsh = BitPat("b???????_?????_?????___001____?????_01001")

  // R-type             |-func7-|-rs2-|-rs1-|-funct3-|--rd-|opcode|
  def fmv_h_x = BitPat("b1111010_?????_?????___000____?????_10100")
}
