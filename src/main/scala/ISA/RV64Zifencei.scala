package ISA

import chisel3.util.BitPat

object RV64Zifencei extends Decoder32Bits {
  // fence              |-func7-|-rs2-|-rs1-|-funct3-|--rd-|opcode|
  def fence_i = BitPat("b???????_?????_?????___001____?????_00011")
}
