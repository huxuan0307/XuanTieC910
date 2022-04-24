package Core.IDU.DecodeTable

import Utils.Bits.BinaryConst.{T, F}
import chisel3._
import ISA.RV64I._
import Core.IDU.Opcode.AluOpcode._
import Core.IDU.Opcode.ShiftLogicOpcode._
import Core.IDU.FuncUnit._

object AluDecodeTable {

  val table = Seq(
    //        Fu   opcode     rs2
    //        |    |       rs1|
    //        |    |    rd |  |
    lui   -> (ALU, LUI, T, F, F),
    addi  -> (ALU, ADD, T, T, F),
    slti  -> (ALU, SLT, T, T, F),
    sltiu -> (ALU, SLT, T, T, F),
    xori  -> (ALU, XOR, T, T, F),
    ori   -> (ALU, OR , T, T, F),
    andi  -> (ALU, AND, T, T, F),
    slli  -> (ALU, SLL, T, T, F),
    srli  -> (ALU, SRL, T, T, F),
    srai  -> (ALU, SRA, T, T, F),
    addiw -> (ALU, ADDW,T, T, F),
    slliw -> (ALU, SLLW,T, T, F),
    srliw -> (ALU, SRLW,T, T, F),
    sraiw -> (ALU, SRAW,T, T, F),
    addw  -> (ALU, ADDW,T, T, T),
    subw  -> (ALU, SUBW,T, T, T),
    sllw  -> (ALU, SLLW,T, T, T),
    srlw  -> (ALU, SRLW,T, T, T),
    sraw  -> (ALU, SRAW,T, T, T),
    add   -> (ALU, ADD, T, T, T),
    sub   -> (ALU, SUB, T, T, T),
    sll   -> (ALU, SLL, T, T, T),
    slt   -> (ALU, SLT, T, T, T),
    sltu  -> (ALU, SLTU,T, T, T),
    xor   -> (ALU, XOR, T, T, T),
    srl   -> (ALU, SRL, T, T, T),
    sra   -> (ALU, SRA, T, T, T),
    or    -> (ALU, OR,  T, T, T),
    and   -> (ALU, AND, T, T, T),
  )
}
