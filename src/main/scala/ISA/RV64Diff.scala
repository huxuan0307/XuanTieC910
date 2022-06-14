package ISA

import chisel3.util.BitPat

object RV64Diff extends Decoder32Bits {
  // Todo: Configurable
  def trapInst: BitPat = BitPat("b0000000_00000_00000_000_1101011")
}
