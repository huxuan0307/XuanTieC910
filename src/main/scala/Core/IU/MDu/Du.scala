package Core.IU.MDu

import Core.Config.XLEN
import Core.IU.Du.MDUbit
import Core.IU.{IduRfPipe0, unitSel}
import Core.{IUConfig, MDUOpType}
import Utils.{HasCircularQueuePtrHelper, LookupTree, SignExt, ZeroExt}
import chisel3._
import chisel3.util._
class DuRegData extends Bundle {
  val pipe0DataVld = Bool()
  val preg = UInt(7.W)
  val data = UInt(64.W)
}

class DuIO extends Bundle {
  val in    = Input(new IduRfPipe0)
  val sel   = Input(new unitSel)
  val out   = Output(new DuRegData)
  val flush = Input(Bool())//l/d会改变数据，所以必须确认存在分支的位置，需要补充一个predict.ROBIdx，而其它EXU只需关注是否在mispred.ROBIdx之后
  //val DivIdle = Output(Bool())
}

class DivIO(val len: Int) extends Bundle {
  val in = Flipped(DecoupledIO(Vec(2, Output(UInt(len.W)))))
  val flush  = Input(Bool())
  //val sign = Input(Bool())
  val out = ValidIO(Output(UInt((len).W)))//todo: Now Quotient, don't need remainder
  val rem = Output(UInt((len).W))
  //val res2 = Output(UInt((len).W))
}

class Radix8Divider(len: Int = 64) extends Module {
  val io = IO(new DivIO(len))
  val s_idle :: s_shift :: s_compute ::  Nil = Enum(3)
  val state = RegInit(s_idle)
  //io.in.ready := (state === s_idle)
  val newReq = (state === s_idle) && io.in.valid
  val (a, b) = (io.in.bits(0), io.in.bits(1))
  val shiftReg = RegInit(0.U((len * 2).W))
  val (hi,lo) = (shiftReg(len * 2-1, len),shiftReg(len - 1, 0))

  val aReg = RegEnable(a, 0.U.asTypeOf(a), newReq)
  val bReg = RegEnable(b, 0.U.asTypeOf(b), newReq)

  val cnt = RegEnable(len.U(log2Up(len+1).W), len.U(log2Up(len+1).W), newReq)//

  when(newReq){
    val canSkipShift = (len.U | Log2(b)) - Log2(a)
    cnt := Mux(canSkipShift >= len.U, len.U, canSkipShift)
    state := s_shift
  }.elsewhen (state === s_shift) {
    shiftReg := aReg << cnt
    state := s_compute
  }.elsewhen (state === s_compute) {
    when(cnt > len.U){
      state := s_idle
    }.elsewhen(cnt === len.U){
      val sr1 = Wire(UInt(len.W))
      val x1enough = hi.asUInt >= bReg.asUInt
      sr1 := Mux(x1enough, hi - bReg, hi)
      shiftReg := Cat(sr1,lo(len-2,0),x1enough)
      state := s_idle
    }.elsewhen(cnt === (len-1).U){
      val sr2 = Wire(UInt(len.W))
      val sr1 = Wire(UInt(len.W))
      val x2enough = hi.asUInt >= bReg.asUInt
      sr2 := Cat(Mux(x2enough, hi - bReg, hi)(len - 2, 0), lo(len-1))
      val x1enough = sr2.asUInt >= bReg.asUInt
      sr1 := Mux(x1enough, sr2 - bReg, sr2)
      shiftReg := Cat(sr1,lo(len-3,0),x2enough,x1enough)
      state := s_idle
    }.elsewhen(cnt === (len-2).U){
      val sr4 = Wire(UInt(len.W))
      val sr2 = Wire(UInt(len.W))
      val sr1 = Wire(UInt(len.W))
      val x4enough = hi.asUInt >= bReg.asUInt
      sr4 := Cat(Mux(x4enough, hi - bReg, hi)(len - 2, 0), lo(len-1))
      val x2enough = sr4.asUInt >= bReg.asUInt
      sr2 := Cat(Mux(x2enough, sr4 - bReg, sr4)(len - 2, 0), lo(len-2))
      val x1enough = sr2.asUInt >= bReg.asUInt
      sr1 := Mux(x1enough, sr2 - bReg, sr2)
      shiftReg := Cat(sr1, lo(len-4,0), x4enough, x2enough, x1enough)
      state := s_idle
    }.otherwise{
      val sr4 = Wire(UInt(len.W))
      val sr2 = Wire(UInt(len.W))
      val sr1 = Wire(UInt(len.W))
      val x4enough = hi.asUInt >= bReg.asUInt
      sr4 := Cat(Mux(x4enough, hi - bReg, hi)(len - 2, 0), lo(len-1))
      val x2enough = sr4.asUInt >= bReg.asUInt
      sr2 := Cat(Mux(x2enough, sr4 - bReg, sr4)(len - 2, 0), lo(len-2))
      val x1enough = sr2.asUInt >= bReg.asUInt
      sr1 := Cat(Mux(x1enough, sr2 - bReg, sr2)(len - 2, 0), lo(len-3))
      shiftReg := Cat(sr1, lo(len-4,0), x4enough, x2enough, x1enough)
      cnt := cnt + 3.U
    }
  }
  val kill = (state=/=s_idle) && io.flush
  when(kill){
    state := s_idle
  }
  io.in.ready := (state === s_idle)
  io.out.valid := RegNext(state) === s_compute && state === s_idle && RegNext(!io.flush)
  io.out.bits := shiftReg(len-1,0)
  //io.quot := shiftReg(len-1,0)
  io.rem := shiftReg(len*2-1,len)//remainder

  //  printf("R8D in.valid %d state %d, a %d b %x\n",io.in.valid,state,a,b)
  //  printf("R8D out.valid %d, out %d rem %d\n",io.out.valid,io.out.bits,io.rem)
  //  printf("R8D cnt %d, in.ready %d\n",cnt,io.in.ready)
  //  printf("????????????????????????????????????????????????????????????????????\n")
}

class Du extends Module with IUConfig with HasCircularQueuePtrHelper {
  val io   = IO(new DuIO)
  val div  = Module(new Radix8Divider(XLEN))
  val pipe1_en = io.sel.gateSel
  //----------------------------------------------------------
  //               Pipe0 EX1 Instruction Data
  //----------------------------------------------------------
  val ex1_pipe = RegEnable(io.in, 0.U.asTypeOf(io.in), pipe1_en)
  val (src1,src2,funcOpType) = (ex1_pipe.src0, ex1_pipe.src1, ex1_pipe.opcode)
  val isDiv = RegInit(false.B)
  val isW   = RegInit(false.B)
  val isDivSign = RegInit(false.B)
  val isRem = RegInit(false.B)
  val src = new MDUbit(UInt(XLEN.W))
  //val ROBIdx = Reg(new ROBPtr)
  val iid  = RegEnable(io.in.iid, 0.U.asTypeOf(io.in.iid), io.sel.sel)
  when(io.sel.sel){
    isDiv     := MDUOpType.isDiv(funcOpType)
    isW       := MDUOpType.isW(funcOpType)
    isDivSign := MDUOpType.isDivSign(funcOpType)
    isRem     := MDUOpType.isRem(funcOpType)
  }
  //in RS_DU, ensure that when io.in.valid, last is done

  def isMinus64(x:UInt):Bool = x(XLEN-1)  //通过补码判断是否为负数
  def isMinus32(x:UInt):Bool = x(32-1)

  div.io.in.valid := io.sel.sel
  div.io.flush := io.flush// TODO rob ptr judge && isAfter(iid,io.mispred_robPtr)

  val quotMinus = RegInit(false.B)
  val remMinus = RegInit(false.B)
  when(io.sel.sel){
    quotMinus := LookupTree(funcOpType, List(
      MDUOpType.div     ->   (isMinus64(src1) ^ isMinus64(src2)),
      MDUOpType.divu    ->   false.B,
      MDUOpType.divw    ->   (isMinus32(src1) ^ isMinus32(src2)),
      MDUOpType.divuw   ->   false.B
    ))//商的符号
    remMinus := LookupTree(funcOpType, List(
      MDUOpType.rem     ->   isMinus64(src1),
      MDUOpType.remu    ->   false.B,
      MDUOpType.remw    ->   isMinus32(src1),
      MDUOpType.remuw   ->   false.B
    ))//余数符号,跟被除数相同,与除数无关
  }//一开始错在没有把looktree写在Regenable里面

  val divInputFunc = (x: UInt) => Mux(isW, Mux(isDivSign, SignExt(x(31,0), XLEN), ZeroExt(x(31,0), XLEN)), x)
  val dividend_abs = LookupTree(funcOpType, List(
    MDUOpType.div     ->   src.single(src1,64),
    MDUOpType.divu    ->   src1,
    MDUOpType.divw    ->   src.half(src1,32),
    MDUOpType.divuw   ->   src1(31,0),
    MDUOpType.rem     ->   src.single(src1,64),
    MDUOpType.remu    ->   src1,
    MDUOpType.remw    ->   src.half(src1,32),
    MDUOpType.remuw   ->   src1(31,0)
  ))
  val divisor_abs = LookupTree(funcOpType, List(
    MDUOpType.div     ->   src.single(src2,64),
    MDUOpType.divu    ->   src2,
    MDUOpType.divw    ->   src.half(src2,32),
    MDUOpType.divuw   ->   src2(31,0),
    MDUOpType.rem     ->   src.single(src2,64),
    MDUOpType.remu    ->   src2,
    MDUOpType.remw    ->   src.half(src2,32),
    MDUOpType.remuw   ->   src2(31,0)
  ))


  div.io.in.bits(0) := dividend_abs//divInputFunc(src1)//
  div.io.in.bits(1) := divisor_abs//divInputFunc(src2)//
  val divby0 = RegEnable(divisor_abs===0.U, false.B, io.sel.sel)
  //val div0w0 = RegEnable((dividend_abs===0.U)&&(divisor_abs===0.U),io.in.fire())
  val divby0res = SignExt("b1111".U, 64)
  //val res0w0 = Mux(isW,divby0res,Cat(ZeroExt("b00".U,32),SignExt("b1111".U,32)))
  val remU = div.io.rem//Mux(quotMinus,divisor_abs - div.io.rem, div.io.rem)//错！被python骗了:若商为负，绝对值为除数绝对值减余数绝对值
  val quot = Mux(quotMinus, -div.io.out.bits, div.io.out.bits)
  val rem = Mux(remMinus,-remU,remU)//余数符号与被除数相同


  val quotres = Mux(isW, SignExt(quot(31,0), 64), quot)
  val remres  = Mux(isW, SignExt(rem(31,0),64),rem)
  //io.out.bits.res := Mux(div0w0,res0w0,Mux(divby0 && !div0w0,divby0res,Mux(isDiv,quotres,remres)))

  //是flush当拍直接kill，不会给出div.io.out.valid
  // io.in.ready   := div.io.in.ready//为了以防万一，停一拍
  //==========================================================
  //                 Write Back to Rbus
  //==========================================================
  io.out.pipe0DataVld := div.io.out.valid
  io.out.preg         := ex1_pipe.dstPreg
  io.out.data         := Mux(divby0,divby0res,Mux(isRem,remres,quotres))

  //  printf("DU0v in.valid %d uop %x instr %x,  \nDU0v io.out %d %x %x\n",io.in.valid,io.in.bits.uop.cf.pc,io.in.bits.uop.cf.instr,io.out.valid,io.out.bits.uop.cf.pc,io.out.bits.uop.cf.instr)
  //  printf("DU1in src1 %d src2 %d, funcOpType %d, divby0 %d, divby0res %x,iores %x,isW %d,isDiv %d\n",src1,src2,funcOpType,divby0,divby0res,io.out.bits.res,isW,isDiv)
  //  printf("DU2S remU %d quotU %d quotS %d quotMinus %d remMinus %d\n",remU,div.io.out.bits,quot,quotMinus,remMinus)
  //  printf("DUres dividend_abs %d %d res %d rem %d state %d\n",dividend_abs,divisor_abs,quotres,remres,io.in.ready)
}


