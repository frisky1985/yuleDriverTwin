//! 内存模型 — 标准 Cortex-M 映射、MPU、Flash 行为、watchpoint、非对齐检测
//!
//! 采用扁平地址空间 + 区域表模型：
//! - Flash（只读代码区，支持擦写仿真）
//! - SRAM（可读可写）
//! - 外设区（读返回 0，写忽略，可挂总线设备回调，见 `peripheral.rs`/`uart.rs`）
//! - 越界访问 → BusFault；MPU 违反 → MemManage；非对齐 → UsageFault

use crate::peripheral::BusDevice;
use crate::system::SystemBlock;

/// Cortex-M 标准内存区域
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryRegionType {
    Code,
    Sram,
    Peripheral,
    ExternalRam,
    Ccm,
    System,
}

/// 内存区域定义
#[derive(Debug, Clone)]
pub struct MemoryRegion {
    pub name: &'static str,
    pub start: u32,
    pub end: u32,
    pub region_type: MemoryRegionType,
    /// 是否允许写
    pub writable: bool,
    /// 是否允许执行
    pub executable: bool,
}

impl MemoryRegion {
    /// 地址是否落在区域内
    pub fn contains(&self, addr: u32) -> bool {
        addr >= self.start && addr < self.end
    }
}

/// MPU 区域保护配置
#[derive(Debug, Clone)]
pub struct MpuRegion {
    pub index: u8,
    pub start: u32,
    pub size: u32,
    pub read: bool,
    pub write: bool,
    pub execute: bool,
    pub privileged_only: bool,
}

impl MpuRegion {
    pub fn contains(&self, addr: u32) -> bool {
        addr >= self.start && addr < self.start.saturating_add(self.size)
    }
}

/// 内存观察点
#[derive(Debug, Clone)]
pub struct Watchpoint {
    pub address: u32,
    pub size: u32,
    pub on_write: bool,
    pub on_read: bool,
}

impl Watchpoint {
    pub fn contains(&self, addr: u32, width: u32) -> bool {
        // 访问区间 [addr, addr+width) 与观察区间有重叠即触发
        let w_end = self.address.saturating_add(self.size);
        let a_end = addr.saturating_add(width);
        addr < w_end && self.address < a_end
    }
}

/// Flash 扇区状态
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashSectorState {
    Erased,
    Written,
    Erasing,
}

/// 内存访问故障原因（与引擎 FaultReason 对应）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryFault {
    /// 地址落在任何区域之外
    BusFault { address: u32 },
    /// MPU 区域保护拒绝
    MemManage { address: u32 },
    /// 非对齐访问（仅 UNALIGN_TRP 使能时）
    UnalignedAccess { address: u32 },
    /// 对只读区域写入
    ReadOnlyWrite { address: u32 },
}

/// 统一内存模型
#[derive(Debug)]
pub struct Memory {
    /// 区域表（决定地址是否合法）
    pub regions: Vec<MemoryRegion>,
    /// Flash 存储（线性字节数组，按区域映射）
    pub flash: Vec<u8>,
    /// SRAM 存储
    pub sram: Vec<u8>,
    /// CCM 存储（可选）
    pub ccm: Vec<u8>,
    /// MPU 区域表（Phase 2）
    pub mpu_regions: Vec<MpuRegion>,
    /// MPU 是否使能（Phase 2）
    pub mpu_enabled: bool,
    /// 非对齐访问陷阱使能（UNALIGN_TRP）（Phase 2）
    pub unalign_trap: bool,
    /// Watchpoint 列表（Phase 2）
    pub watchpoints: Vec<Watchpoint>,
    /// 最近触发的 watchpoint（供 NVIC 调试观测）
    pub watchpoint_hit: Option<u32>,
    /// Flash 擦写行为：擦除后全 0xFF，写入需先擦除
    pub flash_erase_required: bool,
    /// 总线访问周期统计
    pub read_count: u64,
    pub write_count: u64,
    /// 外设区挂接的总线设备（读写路由：命中设备 → 设备处理，否则读 0 / 写忽略）
    pub peripherals: Vec<Box<dyn BusDevice>>,
}

impl Default for Memory {
    fn default() -> Self {
        Self::m4f_default()
    }
}

impl Memory {
    /// 标准 Cortex-M4F (STM32F407VG) 内存布局
    pub fn m4f_default() -> Self {
        Memory {
            regions: vec![
                MemoryRegion { name: "FLASH", start: 0x0000_0000, end: 0x0008_0000, region_type: MemoryRegionType::Code, writable: false, executable: true },
                MemoryRegion { name: "SRAM", start: 0x2000_0000, end: 0x2002_0000, region_type: MemoryRegionType::Sram, writable: true, executable: true },
                MemoryRegion { name: "CCM", start: 0x1000_0000, end: 0x1001_0000, region_type: MemoryRegionType::Ccm, writable: true, executable: true },
                MemoryRegion { name: "PERIPH", start: 0x4000_0000, end: 0x5000_0000, region_type: MemoryRegionType::Peripheral, writable: true, executable: false },
                MemoryRegion { name: "SYSTEM", start: 0xE000_0000, end: 0xE010_0000, region_type: MemoryRegionType::System, writable: true, executable: false },
            ],
            flash: vec![0xFF; 0x0008_0000],
            sram: vec![0; 0x0002_0000],
            ccm: vec![0; 0x0001_0000],
            mpu_regions: Vec::new(),
            mpu_enabled: false,
            unalign_trap: true,
            watchpoints: Vec::new(),
            watchpoint_hit: None,
            flash_erase_required: true,
            read_count: 0,
            write_count: 0,
            peripherals: Vec::new(),
        }
    }

    /// 构造测试用最小内存（flash 4KB + sram 4KB）
    pub fn test_ram() -> Self {
        Memory {
            regions: vec![
                MemoryRegion { name: "FLASH", start: 0x0000_0000, end: 0x0000_1000, region_type: MemoryRegionType::Code, writable: false, executable: true },
                MemoryRegion { name: "SRAM", start: 0x2000_0000, end: 0x2000_1000, region_type: MemoryRegionType::Sram, writable: true, executable: true },
            ],
            flash: vec![0xFF; 0x1000],
            sram: vec![0; 0x1000],
            ccm: Vec::new(),
            mpu_regions: Vec::new(),
            mpu_enabled: false,
            unalign_trap: true,
            watchpoints: Vec::new(),
            watchpoint_hit: None,
            flash_erase_required: true,
            read_count: 0,
            write_count: 0,
            peripherals: Vec::new(),
        }
    }

    /// 查找地址所属区域
    pub fn region_at(&self, addr: u32) -> Option<&MemoryRegion> {
        self.regions.iter().find(|r| r.contains(addr))
    }

    /// 按区域类型取可变存储（避免借用冲突：不持有 &MemoryRegion 引用）
    fn storage_mut_by_type(&mut self, region_type: MemoryRegionType) -> Option<&mut Vec<u8>> {
        match region_type {
            MemoryRegionType::Code => Some(&mut self.flash),
            MemoryRegionType::Sram => Some(&mut self.sram),
            MemoryRegionType::Ccm => Some(&mut self.ccm),
            _ => None,
        }
    }

    fn storage(&self, region: &MemoryRegion) -> Option<&[u8]> {
        match region.region_type {
            MemoryRegionType::Code => Some(&self.flash),
            MemoryRegionType::Sram => Some(&self.sram),
            MemoryRegionType::Ccm => Some(&self.ccm),
            _ => None,
        }
    }

    /// 地址相对区域起始的偏移
    fn offset_in(region: &MemoryRegion, addr: u32) -> usize {
        (addr - region.start) as usize
    }

    /// 检查 MPU 权限（Phase 2）
    fn check_mpu(&self, addr: u32, width: u32, write: bool) -> Result<(), MemoryFault> {
        if !self.mpu_enabled || self.mpu_regions.is_empty() {
            return Ok(());
        }
        // 命中任何 MPU 区域即受保护；未命中默认允许（Cortex-M 默认行为）
        for r in &self.mpu_regions {
            if r.contains(addr) {
                // 访问跨区域边界时按起始地址判定（简化）
                if write && !r.write {
                    return Err(MemoryFault::MemManage { address: addr });
                }
                if !write && !r.read {
                    return Err(MemoryFault::MemManage { address: addr });
                }
                let _ = width;
                return Ok(());
            }
        }
        Ok(())
    }

    /// 检查 watchpoint（Phase 2）
    fn check_watchpoint(&mut self, addr: u32, width: u32, write: bool) {
        for wp in &self.watchpoints {
            let relevant = if write { wp.on_write } else { wp.on_read };
            if relevant && wp.contains(addr, width) {
                self.watchpoint_hit = Some(addr);
                return;
            }
        }
    }

    /// 执行前权限检查（区域 + 对齐 + MPU）
    fn check_access(&self, addr: u32, width: u32, write: bool) -> Result<(), MemoryFault> {
        // 非对齐检测（Phase 2）：word 需 4 对齐，halfword 需 2 对齐
        if self.unalign_trap && width > 1 {
            let alignment = if width == 4 { 3 } else { 1 };
            if addr & alignment != 0 {
                return Err(MemoryFault::UnalignedAccess { address: addr });
            }
        }
        let region = self
            .region_at(addr)
            .ok_or(MemoryFault::BusFault { address: addr })?;
        // 跨区域边界访问视为越界
        if addr + width - 1 >= region.end {
            return Err(MemoryFault::BusFault { address: addr });
        }
        if write && !region.writable {
            return Err(MemoryFault::ReadOnlyWrite { address: addr });
        }
        self.check_mpu(addr, width, write)
    }

    /// 读取 1 字节
    pub fn read_u8(&mut self, addr: u32) -> Result<u8, MemoryFault> {
        self.check_access(addr, 1, false)?;
        self.check_watchpoint(addr, 1, false);
        self.read_count += 1;
        if let Some(v) = self.peripheral_read(addr, 1) {
            return Ok(v as u8);
        }
        let region = self.region_at(addr).unwrap();
        match self.storage(region) {
            Some(s) => Ok(s[Self::offset_in(region, addr)]),
            None => Ok(0), // 外设区简化读 0
        }
    }

    /// 读取 2 字节（小端）
    pub fn read_u16(&mut self, addr: u32) -> Result<u16, MemoryFault> {
        self.check_access(addr, 2, false)?;
        self.check_watchpoint(addr, 2, false);
        self.read_count += 1;
        if let Some(v) = self.peripheral_read(addr, 2) {
            return Ok(v as u16);
        }
        let region = self.region_at(addr).unwrap();
        let off = Self::offset_in(region, addr);
        match self.storage(region) {
            Some(s) => Ok(u16::from_le_bytes([s[off], s[off + 1]])),
            None => Ok(0),
        }
    }

    /// 读取 4 字节（小端）
    pub fn read_u32(&mut self, addr: u32) -> Result<u32, MemoryFault> {
        self.check_access(addr, 4, false)?;
        self.check_watchpoint(addr, 4, false);
        self.read_count += 1;
        if let Some(v) = self.peripheral_read(addr, 4) {
            return Ok(v);
        }
        let region = self.region_at(addr).unwrap();
        let off = Self::offset_in(region, addr);
        match self.storage(region) {
            Some(s) => Ok(u32::from_le_bytes([s[off], s[off + 1], s[off + 2], s[off + 3]])),
            None => Ok(0),
        }
    }

    /// 写入 1 字节
    pub fn write_u8(&mut self, addr: u32, val: u8) -> Result<(), MemoryFault> {
        self.check_access(addr, 1, true)?;
        self.check_watchpoint(addr, 1, true);
        self.write_count += 1;
        if self.peripheral_write(addr, 1, val as u32) {
            return Ok(());
        }
        let region_type = self.region_at(addr).map(|r| r.region_type).unwrap_or(MemoryRegionType::System);
        let off = self.region_at(addr).map(|r| Self::offset_in(r, addr)).unwrap_or(0);
        match self.storage_mut_by_type(region_type) {
            Some(s) => {
                s[off] = val;
                Ok(())
            }
            None => Ok(()), // 外设区写忽略
        }
    }

    /// 写入 2 字节（小端）
    pub fn write_u16(&mut self, addr: u32, val: u16) -> Result<(), MemoryFault> {
        self.check_access(addr, 2, true)?;
        self.check_watchpoint(addr, 2, true);
        self.write_count += 1;
        if self.peripheral_write(addr, 2, val as u32) {
            return Ok(());
        }
        let region_type = self.region_at(addr).map(|r| r.region_type).unwrap_or(MemoryRegionType::System);
        let off = self.region_at(addr).map(|r| Self::offset_in(r, addr)).unwrap_or(0);
        let bytes = val.to_le_bytes();
        match self.storage_mut_by_type(region_type) {
            Some(s) => {
                for (i, b) in bytes.iter().enumerate() {
                    s[off + i] = *b;
                }
                Ok(())
            }
            None => Ok(()),
        }
    }

    /// 写入 4 字节（小端）
    pub fn write_u32(&mut self, addr: u32, val: u32) -> Result<(), MemoryFault> {
        self.check_access(addr, 4, true)?;
        self.check_watchpoint(addr, 4, true);
        self.write_count += 1;
        if self.peripheral_write(addr, 4, val) {
            return Ok(());
        }
        let region_type = self.region_at(addr).map(|r| r.region_type).unwrap_or(MemoryRegionType::System);
        let off = self.region_at(addr).map(|r| Self::offset_in(r, addr)).unwrap_or(0);
        let bytes = val.to_le_bytes();
        match self.storage_mut_by_type(region_type) {
            Some(s) => {
                for (i, b) in bytes.iter().enumerate() {
                    s[off + i] = *b;
                }
                Ok(())
            }
            None => Ok(()),
        }
    }

    /// 直接向区域存储写入字节序列（ELF 加载专用）
    ///
    /// 绕过运行时访问权限检查（Flash 只读区以"烧录"语义直写），不触发 watchpoint，
    /// 不统计读写周期；地址必须完全落在某个区域内，否则返回 BusFault。
    pub fn load_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), MemoryFault> {
        if data.is_empty() {
            return Ok(());
        }
        // 先校验区域与边界（不持有引用跨可变借用）
        let (region_type, off, in_bounds) = {
            let region = self.region_at(addr).ok_or(MemoryFault::BusFault { address: addr })?;
            let off = Self::offset_in(region, addr);
            let in_bounds = addr
                .checked_add(data.len() as u32)
                .map(|end| end <= region.end)
                .unwrap_or(false);
            (region.region_type, off, in_bounds)
        };
        if !in_bounds {
            return Err(MemoryFault::BusFault { address: addr });
        }
        let storage = self
            .storage_mut_by_type(region_type)
            .ok_or(MemoryFault::BusFault { address: addr })?;
        storage[off..off + data.len()].copy_from_slice(data);
        Ok(())
    }

    /// 按宽度读取
    pub fn read(&mut self, addr: u32, width: u32) -> Result<u32, MemoryFault> {
        match width {
            1 => self.read_u8(addr).map(|v| v as u32),
            2 => self.read_u16(addr).map(|v| v as u32),
            4 => self.read_u32(addr),
            _ => Err(MemoryFault::BusFault { address: addr }),
        }
    }

    /// 按宽度写入
    pub fn write(&mut self, addr: u32, width: u32, val: u32) -> Result<(), MemoryFault> {
        match width {
            1 => self.write_u8(addr, val as u8),
            2 => self.write_u16(addr, val as u16),
            4 => self.write_u32(addr, val),
            _ => Err(MemoryFault::BusFault { address: addr }),
        }
    }

    /// Flash 扇区擦除（Phase 2）：整片回 0xFF
    pub fn flash_erase_sector(&mut self, start: u32, size: u32) -> Result<(), MemoryFault> {
        let region = self
            .region_at(start)
            .ok_or(MemoryFault::BusFault { address: start })?;
        if region.region_type != MemoryRegionType::Code {
            return Err(MemoryFault::BusFault { address: start });
        }
        let off = Self::offset_in(region, start);
        let end = (start + size).min(region.end);
        let len = (end - start) as usize;
        self.flash[off..off + len].fill(0xFF);
        Ok(())
    }

    /// 添加 MPU 区域（Phase 2）
    pub fn mpu_add_region(&mut self, region: MpuRegion) {
        self.mpu_regions.push(region);
    }

    /// 添加 watchpoint（Phase 2）
    pub fn watchpoint_add(&mut self, wp: Watchpoint) {
        self.watchpoints.push(wp);
    }

    /// 复位内存状态
    pub fn reset(&mut self) {
        self.sram.fill(0);
        self.flash.fill(0xFF);
        self.watchpoint_hit = None;
        self.mpu_regions.clear();
        self.mpu_enabled = false;
    }

    // ==================== 外设总线设备挂接 ====================

    /// 挂接一个总线设备（如 UART）到外设区；地址窗口命中后读写由设备处理
    pub fn attach_peripheral(&mut self, dev: impl BusDevice + 'static) {
        self.peripherals.push(Box::new(dev));
    }

    /// 外设区读：命中已挂接设备则返回 `Some(value)`（值已按 width 屏蔽），否则 `None`
    /// 注：SYSTEM 区（0xE000_0000-0xE010_0000）同样路由到已挂接设备（FRT-CHIP-02）
    fn peripheral_read(&mut self, addr: u32, width: u32) -> Option<u32> {
        let is_periph = matches!(
            self.region_at(addr).map(|r| r.region_type),
            Some(MemoryRegionType::Peripheral | MemoryRegionType::System)
        );
        if !is_periph {
            return None;
        }
        let dev = self.peripherals.iter_mut().find(|d| {
            addr >= d.base_address() && addr < d.base_address().wrapping_add(d.window_size())
        })?;
        Some(dev.read(addr, width))
    }

    /// 外设区写：命中设备则写入并返回 `true`，否则 `false`（调用方回落写忽略）
    /// 注：SYSTEM 区（0xE000_0000-0xE010_0000）同样路由到已挂接设备（FRT-CHIP-02）
    fn peripheral_write(&mut self, addr: u32, width: u32, val: u32) -> bool {
        let is_periph = matches!(
            self.region_at(addr).map(|r| r.region_type),
            Some(MemoryRegionType::Peripheral | MemoryRegionType::System)
        );
        if !is_periph {
            return false;
        }
        if let Some(dev) = self.peripherals.iter_mut().find(|d| {
            addr >= d.base_address() && addr < d.base_address().wrapping_add(d.window_size())
        }) {
            dev.write(addr, width, val);
            true
        } else {
            false
        }
    }

    /// 按名称获取已挂接外设的类型擦除引用（供外部 downcast 到具体模型）
    pub fn peripheral_mut_by_name(&mut self, name: &str) -> Option<&mut dyn std::any::Any> {
        self.peripherals
            .iter_mut()
            .find(|d| d.name() == name)
            .map(|d| d.as_any_mut())
    }

    /// 获取 SystemBlock（SCB+SysTick）可变引用（未挂接则 None）
    pub fn system_block_mut(&mut self) -> Option<&mut SystemBlock> {
        self.peripheral_mut_by_name(SystemBlock::NAME)
            .and_then(|any| any.downcast_mut::<SystemBlock>())
    }

    /// 周期驱动 SystemBlock（SysTick 递减，FRT-SYS-02/FRT-CHIP-02）；
    /// 返回本周期新挂起的系统异常号（当前仅 SysTick=15），由引擎仲裁消费。
    pub fn tick_system(&mut self, cycles: u64) -> Option<u8> {
        self.system_block_mut().and_then(|sb| sb.tick(cycles))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_write_sram() {
        let mut m = Memory::test_ram();
        m.write_u32(0x2000_0000, 0xDEAD_BEEF).unwrap();
        assert_eq!(m.read_u32(0x2000_0000).unwrap(), 0xDEAD_BEEF);
        m.write_u16(0x2000_0004, 0x1234).unwrap();
        assert_eq!(m.read_u16(0x2000_0004).unwrap(), 0x1234);
        m.write_u8(0x2000_0008, 0xAB).unwrap();
        assert_eq!(m.read_u8(0x2000_0008).unwrap(), 0xAB);
    }

    #[test]
    fn out_of_range_is_bus_fault() {
        let mut m = Memory::test_ram();
        assert_eq!(m.read_u32(0x3000_0000), Err(MemoryFault::BusFault { address: 0x3000_0000 }));
        assert_eq!(m.write_u8(0xFFFF_0000, 1), Err(MemoryFault::BusFault { address: 0xFFFF_0000 }));
    }

    #[test]
    fn flash_readonly_write_faults() {
        let mut m = Memory::test_ram();
        // flash 只读：写触发 ReadOnlyWrite
        assert_eq!(m.write_u8(0x0000_0100, 0x11), Err(MemoryFault::ReadOnlyWrite { address: 0x0000_0100 }));
        // 读 flash 正常
        m.flash[0x200] = 0x7F;
        assert_eq!(m.read_u8(0x0000_0200).unwrap(), 0x7F);
    }

    #[test]
    fn unaligned_detection() {
        let mut m = Memory::test_ram();
        // word 非对齐 → UsageFault（UnalignedAccess）
        assert_eq!(m.read_u32(0x2000_0002), Err(MemoryFault::UnalignedAccess { address: 0x2000_0002 }));
        assert_eq!(m.write_u32(0x2000_0001, 0), Err(MemoryFault::UnalignedAccess { address: 0x2000_0001 }));
        // halfword 非对齐 → 触发
        assert_eq!(m.read_u16(0x2000_0003), Err(MemoryFault::UnalignedAccess { address: 0x2000_0003 }));
        // 关闭 UNALIGN_TRP 后允许（Cortex-M 行为）
        m.unalign_trap = false;
        m.write_u32(0x2000_0002, 0x1122_3344).unwrap();
        assert_eq!(m.read_u32(0x2000_0002).unwrap(), 0x1122_3344);
    }

    #[test]
    fn watchpoint_triggers() {
        let mut m = Memory::test_ram();
        m.watchpoint_add(Watchpoint { address: 0x2000_0010, size: 4, on_write: true, on_read: false });
        m.write_u32(0x2000_0010, 1).unwrap();
        assert_eq!(m.watchpoint_hit, Some(0x2000_0010));
        m.watchpoint_hit = None;
        m.read_u32(0x2000_0010).unwrap(); // 只写观察 → 读不触发
        assert_eq!(m.watchpoint_hit, None);
    }

    #[test]
    fn mpu_write_protect() {
        let mut m = Memory::test_ram();
        m.mpu_enabled = true;
        m.mpu_add_region(MpuRegion {
            index: 0,
            start: 0x2000_0000,
            size: 0x100,
            read: true,
            write: false,
            execute: true,
            privileged_only: false,
        });
        assert_eq!(m.write_u32(0x2000_0000, 0), Err(MemoryFault::MemManage { address: 0x2000_0000 }));
        // 区域外写正常
        m.write_u32(0x2000_0200, 0x55).unwrap();
        assert_eq!(m.read_u32(0x2000_0200).unwrap(), 0x55);
    }

    #[test]
    fn flash_erase_write_behavior() {
        let mut m = Memory::test_ram();
        m.flash[0x300] = 0x5A;
        // FLASH 只读：直接写触发 ReadOnlyWrite
        assert_eq!(m.write_u8(0x0000_0300, 0x00), Err(MemoryFault::ReadOnlyWrite { address: 0x0000_0300 }));
        // 读 flash 正常
        assert_eq!(m.read_u8(0x0000_0300).unwrap(), 0x5A);
        // 擦除后恢复 0xFF
        m.flash_erase_sector(0x0000_0000, 0x1000).unwrap();
        assert_eq!(m.flash[0x300], 0xFF);
        // 擦除后读也恢复
        assert_eq!(m.read_u8(0x0000_0300).unwrap(), 0xFF);
    }

    #[test]
    fn load_bytes_writes_directly() {
        let mut m = Memory::test_ram();
        // Flash 只读区域也能以"烧录"语义直写
        m.load_bytes(0x0000_0100, &[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();
        assert_eq!(m.read_u32(0x0000_0100).unwrap(), 0xEFBE_ADDE);
        // SRAM 正常直写
        m.load_bytes(0x2000_0000, &[1, 2, 3]).unwrap();
        assert_eq!(m.read_u8(0x2000_0000).unwrap(), 1);
        // 空数据不报错
        m.load_bytes(0x2000_0004, &[]).unwrap();
        // 区域外 / 跨边界 → BusFault
        assert_eq!(
            m.load_bytes(0x3000_0000, &[1]),
            Err(MemoryFault::BusFault { address: 0x3000_0000 })
        );
        assert_eq!(
            m.load_bytes(0x0000_0FFE, &[1, 2, 3]),
            Err(MemoryFault::BusFault { address: 0x0000_0FFE })
        );
    }

    #[test]
    fn peripheral_reads_zero() {
        let mut m = Memory::m4f_default();
        assert_eq!(m.read_u32(0x4002_1000).unwrap(), 0);
        m.write_u32(0x4002_1000, 0x1234).unwrap(); // 写忽略
        assert_eq!(m.read_u32(0x4002_1000).unwrap(), 0);
    }

    #[test]
    fn peripheral_routes_to_attached_uart() {
        use crate::uart::CmsdkUart;
        let mut m = Memory::m4f_default();
        m.attach_peripheral(CmsdkUart::new(0x4000_4000));

        // uart_init 序列 + 输出一个字符
        m.write_u32(0x4000_4008, 0).unwrap();
        m.write_u32(0x4000_4010, 115200).unwrap();
        m.write_u32(0x4000_4008, 0x3).unwrap();
        m.write_u32(0x4000_4000, b'A' as u32).unwrap();

        // 窗口内读回（路由到设备）
        assert_eq!(m.read_u32(0x4000_4008).unwrap(), 0x3);
        assert_eq!(m.read_u32(0x4000_4010).unwrap(), 115200);
        assert_eq!(m.read_u32(0x4000_4004).unwrap(), 0); // STATE

        // 捕获输出可经 downcast 查询
        let uart = m
            .peripheral_mut_by_name("CMSDK-APB-UART")
            .unwrap()
            .downcast_mut::<CmsdkUart>()
            .unwrap();
        assert_eq!(uart.output(), b"A");
        assert_eq!(uart.tx_count(), 1);

        // 窗口外外设区仍为默认读 0 / 写忽略
        assert_eq!(m.read_u32(0x4002_1000).unwrap(), 0);
        m.write_u32(0x4002_1000, 0x1234).unwrap();
        assert_eq!(m.read_u32(0x4002_1000).unwrap(), 0);
    }
}
