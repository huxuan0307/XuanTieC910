package ISA

import chisel3.util.BitPat

object RV64Pri extends Decoder32Bits {
  // Todo:
  //  mret, sret, wfi
  def mret = BitPat("b0011000_?????_?????___000____?????_11100")
}
