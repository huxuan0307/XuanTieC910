package Core.IS

import Core.ExceptionConfig._
import Core.IntConfig._
import Core.VectorUnitConfig._
import chisel3._
import chisel3.util._

trait VfiqConfig {
  def NumSrcVf = 4
}

object VfiqConfig extends VfiqConfig

class VfiqEntryData extends Bundle with VfiqConfig {

}
