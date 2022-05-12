package Core.IU.MDu

import Core.IU.Du.MDUbit
import Core.IU.{IduRfPipe1, unitSel}
import Core.IUConfig
import Utils.{LookupTree, SignExt}
import chisel3._
import chisel3.util._

object MDUOpType {
  def mul    = "b0000".U
  def mulh   = "b0001".U
  def mulhsu = "b0010".U
  def mulhu  = "b0011".U
  def div    = "b0100".U
  def divu   = "b0101".U
  def rem    = "b0110".U
  def remu   = "b0111".U

  def mulw   = "b1000".U
  def divw   = "b1100".U
  def divuw  = "b1101".U
  def remw   = "b1110".U
  def remuw  = "b1111".U

  def isDiv(op: UInt) = op(2)
  def isDivSign(op: UInt) = isDiv(op) && !op(0)
  def isW(op: UInt) = op(3)
  def isRem(op: UInt) = op(2) && op(1)
}


class MuRegData extends Bundle with IUConfig {
  val dataVld  = Bool() // TODO fatal diferent, XT910 has 2 valid, in ex2 & ex3, but there only has 1 valid ,which means mu done
  val preg     = UInt(7.W)
  val data     = UInt(XLEN.W)
}
class MUIO extends Bundle {
  val in    = Input(new IduRfPipe1)
  val sel   = Input(new unitSel)
  val out   = Output(new MuRegData)
  val flush = Input(Bool())
}

class MulIO(val len: Int) extends Bundle {
  val in = Flipped(ValidIO(Vec(2, Output(UInt(len.W)))))
  val flush  = Input(Bool())
  val out = ValidIO(Output(UInt((len * 2).W)))
}

class WTMultiplier extends Module {
  val io = IO(new MulIO(64))
  val (a, b) = (io.in.bits(0), io.in.bits(1))

  //level 0
  val branch0 = Wire(Vec(64, UInt(64.W)))
  for(i <- 0 until 64){
    branch0(i) := Mux(a(i),b,0.U)
  }
  //level 1
  val branch1 = Wire(Vec(32, UInt(66.W)))
  for(i <- 0 until 32){
    branch1(i) := Cat(0.U(1.W),branch0(2*i+1),0.U(1.W)) + Cat(0.U(2.W),branch0(2*i))
  }
  //level2
  val branch2 = Wire(Vec(16, UInt(68.W)))
  for(i <- 0 until 16){
    branch2(i) := Cat(branch1(2*i+1),0.U(2.W)) + Cat(0.U(2.W),branch1(2*i))
  }
  //level3
  val branch3 = Wire(Vec(8, UInt(72.W)))
  for(i <- 0 until 8){
    branch3(i) := Cat(branch2(2*i+1),0.U(4.W)) + Cat(0.U(4.W),branch2(2*i))
  }
  val pip = RegEnable(branch3, io.in.valid)
  //level4
  val branch4 = Wire(Vec(4, UInt(80.W)))
  for(i <- 0 until 4){
    branch4(i) := Cat(pip(2*i+1),0.U(8.W)) + Cat(0.U(8.W),pip(2*i))
  }
  //level5
  val branch5 = Wire(Vec(2, UInt(96.W)))
  for(i <- 0 until 2){
    branch5(i) := Cat(branch4(2*i+1),0.U(16.W)) + Cat(0.U(16.W),branch4(2*i))
  }
  //level6
  val res = Cat(branch5(1),0.U(32.W)) + Cat(0.U(32.W),branch5(0))
  io.out.bits :=  res
  io.out.valid := RegNext(io.in.valid)

  //  printf("WT resU %d a %d b %d valid %d\n",res,a,b,io.out.valid)
  //  printf("xxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n")
}

class Mu extends Module with IUConfig {
  val io   = IO(new MUIO)
  val mul  = Module(new WTMultiplier)
  //----------------------------------------------------------
  //               Pipe0 EX1 Instruction Data
  //----------------------------------------------------------
  val pipe1_en = io.sel.gateSel
  val ex1_pipe = RegEnable(io.in, pipe1_en)
  val (src1,src2,funcOpType) = (ex1_pipe.src0, ex1_pipe.src1, ex1_pipe.func)
  val src = new MDUbit(UInt(XLEN.W))
  val lastOp = RegEnable(funcOpType, io.sel.sel)
  //val isDiv = MDUOpType.isDiv(funcOpType)
  //val isDivSign = MDUOpType.isDivSign(funcOpType)
  val isW = RegEnable(MDUOpType.isW(funcOpType),io.sel.sel)
  def isMinus(x:UInt):Bool = x(XLEN-1)  //通过补码判断是否为负数
  mul.io.in.valid   := io.sel.sel   //如果是乘法则进入
  mul.io.flush := io.flush
  val (resMinus:Bool) = RegEnable(LookupTree(funcOpType, List(
    MDUOpType.mul     ->   (isMinus(src1) ^ isMinus(src2)),
    MDUOpType.mulh    ->   (isMinus(src1) ^ isMinus(src2)),
    MDUOpType.mulhsu  ->   isMinus(src1),
    MDUOpType.mulhu   ->   false.B,
    MDUOpType.mulw    ->   (isMinus(src1) ^ isMinus(src2))
  )),io.sel.sel)
  //  val (mul.io.in.bits) = LookupTree(funcOpType, List(
  //    MDUOpType.mul     ->   (src1, src2),
  //    MDUOpType.mulh    ->   (src.single(src1), src.single(src2)),
  //    MDUOpType.mulhsu  ->   (src.single(src1), src2),
  //    MDUOpType.mulhu   ->   (src1, src2),
  //    MDUOpType.mulw    ->   (src1, src2)
  //  ))
  mul.io.in.bits(0) := LookupTree(funcOpType, List(
    MDUOpType.mul     ->   src.abs(src1,64),
    MDUOpType.mulh    ->   src.abs(src1,64),
    MDUOpType.mulhsu  ->   src.abs(src1,64),
    MDUOpType.mulhu   ->   src1,
    MDUOpType.mulw    ->   src.abs(src1,64)
  ))
  mul.io.in.bits(1) := LookupTree(funcOpType, List(
    MDUOpType.mul     ->   src.abs(src2,64),
    MDUOpType.mulh    ->   src.abs(src2,64),
    MDUOpType.mulhsu  ->   src2,
    MDUOpType.mulhu   ->   src2,
    MDUOpType.mulw    ->   src.abs(src2,64)
  ))
  val res1 = Mux(resMinus, -mul.io.out.bits, mul.io.out.bits)
  val res = LookupTree(lastOp, List(
    MDUOpType.mul     ->   res1(63,0),
    MDUOpType.mulh    ->   res1(127,64),
    MDUOpType.mulhsu  ->   res1(127,64),
    MDUOpType.mulhu   ->   res1(127,64),
    MDUOpType.mulw    ->   res1(31,0)
  ))
  //==========================================================
  //                        Result
  //==========================================================
  io.out.data     := Mux(isW, SignExt(res(31,0), 64), res)//all val about out need RegEnable
  io.out.dataVld  := mul.io.out.valid   //RegNext(io.in.valid && !io.flush) && mul.io.out.valid //做了冗余
  io.out.preg     := ex1_pipe.dstPreg

  //  printf("MU0v in.valid %d pc %x instr %x, io.out %d %x %x\n",io.in.valid,io.in.bits.uop.cf.pc,io.in.bits.uop.cf.instr,io.out.valid,io.out.bits.uop.cf.pc,io.out.bits.uop.cf.instr)
  //  printf("MU1in src1 %d src2 %d, funcOpType %d,lastOp %d\n",src1,src2,funcOpType,lastOp)
  //  printf("MU2S out.valid %d resU %d resS %d outres %d\n",mul.io.out.valid,mul.io.out.bits,res1,io.out.bits.res)
  //  printf("=================================================================\n")
}
