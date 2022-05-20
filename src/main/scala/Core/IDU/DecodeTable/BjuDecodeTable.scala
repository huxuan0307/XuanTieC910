package Core.IDU.DecodeTable

import Utils.Bits.BinaryConst.{F, T}
import chisel3._
import chisel3.util.BitPat
import ISA.RV64I._
import Core.IDU.Opcode.BjuOpcode._
import Core.IDU.FuncUnit._

object BjuDecodeTable {
  val table : Array[(BitPat, List[UInt])] = Array(
    //            Fu   opcode      rs2
    //            |    |        rs1|
    //            |    |     rd |  |
    jal   -> List(BJU, JAL , F, F, F),
    jalr  -> List(BJU, JALR, F, T, F),
    beq   -> List(BJU, BEQ , F, T, T),
    bne   -> List(BJU, BEQ , F, T, T),
    blt   -> List(BJU, BEQ , F, T, T),
    bge   -> List(BJU, BEQ , F, T, T),
    bltu  -> List(BJU, BEQ , F, T, T),
    bgeu  -> List(BJU, BEQ , F, T, T),
  )
}
