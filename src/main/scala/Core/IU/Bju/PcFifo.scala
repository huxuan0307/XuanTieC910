package Core.IU.Bju

import Core.IUConfig.PcFifoLen
import Utils.{CircularQueuePtr, HasCircularQueuePtrHelper}
import chisel3._
import chisel3.util._

class PcFifoPtr extends CircularQueuePtr[PcFifoPtr](PcFifoLen){
  override def cloneType = (new PcFifoPtr).asInstanceOf[this.type]
}

object PcFifoPtr {
  def apply(f: Bool, v:UInt): PcFifoPtr = {
    val ptr = Wire(new PcFifoPtr)
    ptr.flag := f
    ptr.value := v
    ptr
  }
}

class IduWriteIO extends Bundle{
  val writePcfifo  = Vec(2,Input(new IfuPredStore))
  val alloPid      = Vec(2, Output(UInt(5.W)))
}
class PidIO extends Bundle{
  val read   = Input(UInt(5.W))
  val write  = Output(UInt(5.W))
}
class BjuRwIO extends Bundle {
  val readPcfifo  = Output(new IfuPredStore)
  val pid = new PidIO
  val writePcfifo = Valid(Input(new PredCheckRes))
}
class RobReadIO extends Bundle{
  val ifu_pred = Output(new BhtPredDataForward)
  val bht_check = Output(new PredCheckRes)
  val dealloPid = Input(UInt(5.W))
}
class PcFifoIO extends Bundle{
  val iduWrite  = Valid(new IduWriteIO) // valid to out - pcfifo is full ? bht can not in : bht can in
  val bjuRw     = new BjuRwIO
  val robRead   = DecoupledIO(new RobReadIO)
}

class PcFifo extends Module with HasCircularQueuePtrHelper{
  // PcFifo storage 3 data: chk_idx, bht_pred, jmp_mispred
  val io = IO(new PcFifoIO())
  val ifu_forward_data = io.iduWrite.bits.writePcfifo // how to name this data? just use same name whit XT910, ct_io_bju_fio.v @1677 - 1693

  var fifo_tab_pred  = RegInit(VecInit(Seq.fill(PcFifoLen)( 0.U.asTypeOf(new IfuPredStore) )))
  var fifo_tab_check = RegInit(VecInit(Seq.fill(PcFifoLen)( 0.U.asTypeOf(new PredCheckRes) )))

  val headPtr = RegInit(PcFifoPtr(false.B, 0.U))
  val tailPtr = RegInit(PcFifoPtr(true.B,  0.U))
  val is_full = distanceBetween(tailPtr, headPtr) < 2.U
  io.iduWrite.valid := is_full
  // idu write in & allocate ptr
  val allocatePtrs = (0 until 2).map(i => headPtr + i.U) // is_full to control allo_ptr
  for(i <- 0 until 2){
    io.iduWrite.bits.alloPid(i) := allocatePtrs(i).value
  }
  for(i <- 0 until 2){
    val offset = i.U //if(i == 0) 0.U else PopCount(ifu_forward_data(i).en ) // TODO ?
    val ptr = tailPtr + offset
    val idx = ptr.value
    fifo_tab_pred(idx) := ifu_forward_data(i)
  }
  // bju read by pid
  val bju_read_pid = io.bjuRw.pid.read
  val readData = fifo_tab_pred(bju_read_pid) // VecInit(bju_read_pid.map(addr => fifo_tab_pred(addr)))
  // bju write by pid
  val bju_write_pid = io.bjuRw.pid.write
  when(io.bjuRw.writePcfifo.valid){
    fifo_tab_check(bju_write_pid) := io.bjuRw.writePcfifo.bits
  }
  // rob read & deallocate ptr
  when(io.robRead.fire){
    io.robRead.bits.bht_check :=  fifo_tab_check(tailPtr.value)
    io.robRead.bits.ifu_pred  :=  fifo_tab_pred(tailPtr.value)
    val tailPtrNext = tailPtr + 1.U
    tailPtr := tailPtrNext
  }
}
