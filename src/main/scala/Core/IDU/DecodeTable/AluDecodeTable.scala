package Core.IDU.DecodeTable

import Core.IDU.FuncUnit._
import Core.IDU.Opcode.AluOpcode._
import ISA.RV64Diff.trapInst
import ISA.RV64I._
import Utils.Bits.BinaryConst.{F, T}
import chisel3._
import chisel3.util.BitPat

object AluDecodeTable {

  val table : Array[(BitPat, List[UInt])] = Array(
    //            Fu   opcode     rs2
    //            |    |       rs1|
    //            |    |    rd |  |
    lui   -> List(ALU, LUI, T, F, F),
    auipc -> List(ALU, ADD, T, F, F),
    addi  -> List(ALU, ADD, T, T, F),
    slti  -> List(ALU, SLT, T, T, F),
    sltiu -> List(ALU, SLT, T, T, F),
    xori  -> List(ALU, XOR, T, T, F),
    ori   -> List(ALU, OR , T, T, F),
    andi  -> List(ALU, AND, T, T, F),
    slli  -> List(ALU, SLL, T, T, F),
    srli  -> List(ALU, SRL, T, T, F),
    srai  -> List(ALU, SRA, T, T, F),
    addiw -> List(ALU, ADDW,T, T, F),
    slliw -> List(ALU, SLLW,T, T, F),
    srliw -> List(ALU, SRLW,T, T, F),
    sraiw -> List(ALU, SRAW,T, T, F),
    addw  -> List(ALU, ADDW,T, T, T),
    subw  -> List(ALU, SUBW,T, T, T),
    sllw  -> List(ALU, SLLW,T, T, T),
    srlw  -> List(ALU, SRLW,T, T, T),
    sraw  -> List(ALU, SRAW,T, T, T),
    add   -> List(ALU, ADD, T, T, T),
    sub   -> List(ALU, SUB, T, T, T),
    sll   -> List(ALU, SLL, T, T, T),
    slt   -> List(ALU, SLT, T, T, T),
    sltu  -> List(ALU, SLTU,T, T, T),
    xor   -> List(ALU, XOR, T, T, T),
    srl   -> List(ALU, SRL, T, T, T),
    sra   -> List(ALU, SRA, T, T, T),
    or    -> List(ALU, OR,  T, T, T),
    and   -> List(ALU, AND, T, T, T),
    trapInst -> List(ALU, ADD, F, T, T)
  )
}
