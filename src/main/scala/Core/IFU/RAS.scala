package Core.IFU
import Core.{Config, CoreBundle}
import Core.utils._
import chisel3._
import chisel3.util._

class RASPtr extends CircularQueuePtr[RASPtr](16) with HasCircularQueuePtrHelper{
  override def cloneType = (new RASPtr).asInstanceOf[this.type]
}

class RASEntry extends Bundle with Config{
  val retAddr = UInt(VAddrBits.W)
  val ctr = UInt(10.W) // layer of nested call functions
}
object RASEntry {
  def apply(retAddr: UInt, ctr: UInt): RASEntry = {
    val e = Wire(new RASEntry)
    e.retAddr := retAddr
    e.ctr := ctr
    e
  }
}
class RASUpdateIO extends CoreBundle {
  val iscall = Input(Bool())
  val isret  = Input(Bool())
  val target = Input(UInt(VAddrBits.W))
}
class RASIO extends CoreBundle {
  val ifu_update = new RASUpdateIO
  val rtu_update = new RASUpdateIO
  val target = Output(UInt(VAddrBits.W))
  val flush  = Input(Bool())
  val ras_flush = Input(Bool())
}
class RAS extends Module with Config {
  val io = IO(new RASIO)
  val stack = RegInit(VecInit(Seq.fill(16)(0.U.asTypeOf(new RASEntry))))
  val sp_ptr = RegInit(0.U.asTypeOf(new RASPtr))
  val top = stack(sp_ptr.value)
  io.target := top.retAddr

  when(io.ifu_update.iscall && io.ifu_update.isret){
    when(top.ctr===1.U){
      stack(sp_ptr.value) := RASEntry(io.ifu_update.target, 1.U)
    }.elsewhen(top.retAddr=/=io.ifu_update.target){
      stack(sp_ptr.value)       := RASEntry(top.retAddr, top.ctr - 1.U)
      stack(sp_ptr.value + 1.U) := RASEntry(io.ifu_update.target, 1.U)
      sp_ptr.value := sp_ptr.value + 1.U
    }
  }
  when(io.ifu_update.iscall && !io.ifu_update.isret){
    when(top.retAddr===io.ifu_update.target){
      stack(sp_ptr.value) := RASEntry(top.retAddr, top.ctr + 1.U)
    }.otherwise{
      stack(sp_ptr.value + 1.U) := RASEntry(io.ifu_update.target, 1.U)
      sp_ptr.value := sp_ptr.value + 1.U
    }
  }
  when(io.ifu_update.isret && !io.ifu_update.iscall){
    when(top.ctr===1.U){
      sp_ptr.value := sp_ptr.value - 1.U
    }.otherwise{
      stack(sp_ptr.value) := RASEntry(top.retAddr, top.ctr - 1.U)
    }
  }

  val stack_commit = RegInit(VecInit(Seq.fill(16)(0.U.asTypeOf(new RASEntry))))//新建第二个表
  val sp_commit = RegInit(0.U.asTypeOf(new RASPtr))
  val top_commit = stack_commit(sp_commit.value)

  when(io.rtu_update.iscall && io.rtu_update.isret){
    when(top_commit.ctr===1.U){
      stack_commit(sp_commit.value) := RASEntry(io.rtu_update.target, 1.U)
    }.elsewhen(top_commit.retAddr=/=io.rtu_update.target){
      stack_commit(sp_commit.value)       := RASEntry(top_commit.retAddr, top_commit.ctr - 1.U)
      stack_commit(sp_commit.value + 1.U) := RASEntry(io.rtu_update.target, 1.U)
      sp_commit.value := sp_commit.value + 1.U
    }
  }
  when(io.rtu_update.iscall && !io.rtu_update.isret){
    when(top_commit.retAddr===io.rtu_update.target){
      stack_commit(sp_commit.value) := RASEntry(top_commit.retAddr, top_commit.ctr + 1.U)
    }.otherwise{
      stack_commit(sp_commit.value + 1.U) := RASEntry(io.rtu_update.target, 1.U)//指针+1，计数层置为1
      sp_commit.value := sp_commit.value + 1.U
    }
  }
  when(io.rtu_update.isret && !io.rtu_update.iscall){
    when(top_commit.ctr===1.U){
      sp_commit.value := sp_commit.value - 1.U//已调用完毕，指针回退
    }.otherwise{
      stack_commit(sp_commit.value) := RASEntry(top_commit.retAddr, top_commit.ctr - 1.U)//不是第一层，地址写入commit表，计数层减1
    }
  }

  when(io.flush){
    for(i <- 0 until 16){
      stack(i) := stack_commit(i)//冲刷时，stack根据commit更新
    }
    sp_ptr.value := sp_commit.value

    when(io.rtu_update.iscall){
      when(top_commit.retAddr===io.rtu_update.target){
        stack(sp_commit.value).ctr := stack_commit(sp_commit.value).ctr + 1.U
      }.otherwise{
        sp_ptr.value := sp_commit.value + 1.U
        stack(sp_commit.value + 1.U).retAddr := io.rtu_update.target
        stack(sp_commit.value + 1.U).ctr := 1.U
      }
    }
  }

  when(io.ras_flush){
    sp_ptr.value := 0.U
    sp_commit.value := 0.U
    for(i <- 0 until 16){
      stack_commit(i).retAddr := 0.U
      stack_commit(i).ctr := 0.U
      stack(i).retAddr := 0.U
      stack(i).ctr := 0.U
    }
  }
}
