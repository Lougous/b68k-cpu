--
-- glue CPLD for b68k CPU board
--   https://github.com/Lougous/b68k-cpu
--
-- target: Altera EPM7128S
-- 

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity glue is

  port (

    RSTn : in  std_logic;
    GCLK : in  std_logic;
    
    CPU_A      : in  std_logic_vector(23 downto 1);
    CPU_ASn    : in  std_logic;
    CPU_LDSn   : in  std_logic;
    CPU_UDSn   : in  std_logic;
    CPU_RWn    : in  std_logic;
    CPU_FC2    : in  std_logic;
    CPU_BERRn  : out std_logic;
    CPU_D      : in std_logic_vector(15 downto 8);
    CPU_DTACKn : out std_logic;
    CPU_VPAn   : out std_logic;

    M_A     : out std_logic_vector(10 downto 0);
    M_CASLn : out std_logic;
    M_CASUn : out std_logic;
    M_RASn  : out std_logic;
    M_WEn   : out std_logic;
    
    B_WEn     : out std_logic;
    B_ACKn    : in  std_logic;
    B_A0      : out std_logic;
    G_OEn_LO  : out std_logic;
    G_CSn     : out std_logic;
    G_OEn_HI  : out std_logic;
    G_DIR     : out std_logic;
    G_OEn     : out std_logic

    );

end entity glue;

architecture rtl of glue is

  -- registers
  signal reg_pmu_base : std_logic_vector(22 downto 12);
  signal reg_pmu_mask : std_logic_vector(21 downto 12);
  signal reg_update   : std_logic;
 
  -- output address/data mux control
  signal a_sel_address : std_logic;
  signal a_sel_hilo    : std_logic;

  signal io_address : std_logic_vector(2 downto 0);
  
  -- states
  signal s_0      : std_logic;
  signal s_cpu    : std_logic;
  signal s_dram   : std_logic;
  signal s_ref    : std_logic;
  signal s_io_1   : std_logic;
  signal s_io_2   : std_logic;
  signal s_io_3_r : std_logic;
  signal s_io_3_w : std_logic;
  signal s_io_4   : std_logic;
  signal s_io_5   : std_logic;
  signal s_io_6   : std_logic;
  signal s_io_7   : std_logic;

  signal data_cs_n : std_logic;
  signal addr_cs_n : std_logic;

  signal dly : std_logic;
  signal even_cycle : std_logic;

  signal VPAn_mask  : std_logic;
  signal BERRn_mask : std_logic;
  signal DTACKn_sr  : std_logic_vector(1 downto 0);

  -- refresh
  signal rf_cnt : unsigned(8 downto 0);
  signal rf_req : std_logic;

  -- DMA
  signal dma_in_progress : std_logic;

  -- DRAM control
  signal casl_n : std_logic;
  signal casu_n : std_logic;
  signal casl_fall_n : std_logic;
  signal casu_fall_n : std_logic;
  signal ras_n  : std_logic;

  -- IO bus
  signal B_ACKn_meta : std_logic;
  signal B_ACKn_cdc  : std_logic;

  signal s_io_1_fall : std_logic;

  -- PMU
  signal usr_phy_ad       : std_logic_vector(22 downto 12);
  signal pmu_access_error : std_logic;

begin  -- architecture rtl

  usr_phy_ad <= (CPU_A(22 downto 12) and ("0" & reg_pmu_mask)) or reg_pmu_base;
  
  -- address mux, shared between DRAM and possibly IO bus address/data lines
  M_A(7 downto 0) <=
    ----------------------------------------------------------------------------
    -- IO/DMA bus ops
    ----------------------------------------------------------------------------
    std_logic_vector(rf_cnt(8 downto 1)) when dma_in_progress = '1' and a_sel_hilo = '0' else
    ----------------------------------------------------------------------------
    -- IO bus or DRAM ops
    ----------------------------------------------------------------------------
    -- low address
    CPU_A(8 downto 1) when a_sel_address = '1' and a_sel_hilo = '0' else
    -- high address, supervisor
    CPU_A(16 downto 9) when a_sel_address = '1' and a_sel_hilo = '1' and (CPU_FC2 = '1' or CPU_A(23) = '1') else
    -- high address, user
    usr_phy_ad(16 downto 12) & CPU_A(11 downto 9) when a_sel_address = '1' and a_sel_hilo = '1'  else
    ----------------------------------------------------------------------------
    -- IO bus ops data (MSB)
    ----------------------------------------------------------------------------
    CPU_D;

  io_address <= CPU_A(19 downto 17);
  
  -- address mux, shared between DRAM and possible IO bus chip select section
  M_A(10 downto 8) <=
    ----------------------------------------------------------------------------
    -- IO bus ops
    ----------------------------------------------------------------------------
    -- IO chip select 1 (IO bus address phase)
    "011" when s_io_1 = '1' or s_io_2 = '1' else
    -- IO chip select (IO bus data phase)
    io_address when a_sel_address = '0' else
    ----------------------------------------------------------------------------
    -- DRAM ops
    ----------------------------------------------------------------------------
    -- low address for supervisor (DRAM column or IO chip select)
    CPU_A(21) & CPU_A(18 downto 17) when a_sel_hilo = '0' and CPU_FC2 = '1' else
    -- low address for user (DRAM column)
    usr_phy_ad(21) & usr_phy_ad(18 downto 17) when a_sel_hilo = '0' and CPU_FC2 = '0' else
    -- hi address for supervisor (DRAM raw)
    CPU_A(22) & CPU_A(20 downto 19) when a_sel_hilo = '1' and CPU_FC2 = '1' else
    -- hi address for user (DRAM raw)
    -- a_sel_hilo = '1' and CPU_FC2 = '0' 
    usr_phy_ad(22) & usr_phy_ad(20 downto 19);

  G_DIR <= not CPU_RWn;

  pmu_check : process (GCLK) is
  begin
    if falling_edge(GCLK) then
      if CPU_FC2 = '1' then
        -- supervisor = all
        pmu_access_error <= '0';
      elsif CPU_A(23) = '0' and CPU_A(22) = '0' and unsigned(CPU_A(21 downto 12) and (not reg_pmu_mask)) = 0 then
        -- user, DRAM
        pmu_access_error <= '0';
      elsif CPU_A(23) = '1' and io_address = "100" then
        -- user, IOs
        pmu_access_error <= '0';
      else
        pmu_access_error <= '1';
      end if;
    end if;
  end process pmu_check;
  
  process (RSTn, GCLK) is
  begin
    if RSTn = '0' then
      BERRn_mask <= '1';
      VPAn_mask  <= '1';
      DTACKn_sr  <= "11";

      M_WEn <= '1';

      B_WEn  <= '1';
      B_A0   <= '0';

      G_OEn_HI <= '1';
      G_OEn_LO <= '1';
      G_OEn    <= '1';
      
      s_0      <= '1';
      s_cpu    <= '0';
      s_dram   <= '0';
      s_ref    <= '0';
      s_io_1   <= '0';
      s_io_2   <= '0';
      s_io_3_r <= '0';
      s_io_3_w <= '0';
      s_io_4   <= '0';
      s_io_5   <= '0';
      s_io_6   <= '0';
      s_io_7   <= '0';

      data_cs_n <= '1';
      addr_cs_n <= '1';
      
      dly <= '0';
      even_cycle <= '0';

      rf_cnt <= (others => '1');
      rf_req <= '1';

      casl_n <= '1';
      casu_n <= '1';
      ras_n  <= '1';

      a_sel_hilo <= '1';
      a_sel_address <= '1';

      -- default: no offset/mask
      reg_pmu_base <= (others => '0');
      reg_pmu_mask <= (others => '1');
      
      reg_update   <= '0';

      dma_in_progress <= '1';

    elsif rising_edge(GCLK) then

      reg_update <= '0';

      DTACKn_sr <= '1' & DTACKn_sr(1);

      --------------------------------------------------------------------------
      -- states
      --------------------------------------------------------------------------
      if s_0 = '1' then
        M_WEn <= '1';
        B_WEn <= '1';
        dly <= '0';

        if rf_req = '1' and CPU_ASn = '0' and CPU_RWn = '1' then
          -- go for refresh
          rf_req <= '0';

          -- CAS first
          casl_n <= '0';
          casu_n <= '0';

          -- state
          s_0   <= '0';
          s_ref <= '1';
          
        elsif dma_in_progress = '1' then
          if CPU_ASn = '0' then
            ras_n    <= '0';
            s_0      <= '0';
            s_io_3_r <= '1';
          end if;
          
        elsif CPU_ASn = '0' then
          -- CPU access
          s_0 <= '0';
          
          if pmu_access_error = '0' then

            if CPU_A(23) = '0' then
              -- access to DRAM
              a_sel_hilo <= '0';
          
              M_WEn <= CPU_RWn;

              DTACKn_sr <= "00";

              -- state
              s_dram <= '1';
            else
              -- access to IO or internal registers
              ras_n  <= '0';

              G_OEn_HI <= '0';
        
              if io_address = "111" then
                -- two valid options, both valid in supervisor mode only
                --   PMU registers (write only)
                --   CPU space (interrupt vector)
                --   (all address bits are set to 1 during CPU space access)
                if CPU_A(8) = '0' and CPU_RWn = '0' then
                  -- write to PMU registers (write only registers)
                  DTACKn_sr <= "00";
              
                  reg_update <= '1';
                elsif CPU_A(8) = '1' and CPU_RWn = '1' then
                  -- consider here supervisor is wise enough to not do read
                  -- access to this locations, so it has to be CPU space access (interrupt
                  -- vector)
                  -- VPA# is used for autovector; alternatively BERRn can be used
                  -- here (with DTACK#) to redirect all interrupts to entry 0x60
                  -- (spurious interrupt)
                  VPAn_mask <= '0';
                  -- note: DTACK# and VPA# (AVEC#) should never be simultaneously asserted.
                else 
                  -- bus error
                  BERRn_mask <= '0';
                end if;
              
                -- state
                s_cpu <= '1';
              else
                -- IO bus access
                addr_cs_n <= '0';

                s_0    <= '0';
                s_io_1 <= '1';
              end if;
            end if;
          else
            -- bad access, bus error
            BERRn_mask <= '0';
            s_cpu <= '1';
          end if;
        end if;
      end if;

      if s_cpu = '1' then
        -- CPU memory access, neither DRAM nor Bus
        ras_n  <= '1';

        -- hold VPAn/BERRn (if so) until ASn de-asserted
        if CPU_ASn = '1' then
          VPAn_mask   <= '1';
          BERRn_mask  <= '1';
          s_cpu <= '0';
          s_0 <= '1';
        end if;
      end if;
        
      if s_io_1 = '1' then
        -- Bus address cycle, MSB
        -- first sub-cycle of a word access - works for write cycles only
        even_cycle <= (not CPU_LDSn) and (not CPU_UDSn) and (not CPU_RWn);

        -- next cycle shows address LSB
        a_sel_hilo <= '0';
        s_io_1 <= '0';
        s_io_2 <= '1';
      end if;

      if s_io_2 = '1' then
        -- Bus address cycle, LSB

        -- next cycle will show data on Bus
        a_sel_address <= '0';
      
        s_io_2 <= '0';

        addr_cs_n <= '1';

        -- next cycle is turnover, disable HI+LO
        G_OEn_HI <= '1';

        B_WEn <= CPU_RWn;  -- DMA during CPU read operation

        -- note: DMA during CPU read operation
        if CPU_RWn = '1' then
          -- read 
          s_io_3_r <= '1';
        else
          -- write
          s_io_3_w <= '1';
        end if;
          
      end if;

      if s_io_3_r = '1' then
        -- Bus turnover cycle, read cycle

        -- next cycle is data
        -- UM 5.1.1/5.1.2 : When A0 equals zero, the upper data strobe is
        -- issued; when A0 equals one, the lower data strobe is issued
        B_A0     <= CPU_UDSn;

        -- data from bus to both bytes of CPU data
        G_OEn_LO <= '0';
        G_OEn    <= '0';

        data_cs_n <= '0';

        s_io_3_r <= '0';
        s_io_4   <= '1';
      end if;

      if s_io_3_w = '1' then
        -- Bus turnover cycle, write cycle
        if (CPU_LDSn = '1') or (even_cycle = '1') then
          -- byte access even address or word access first cycle (even byte)
          -- write MSB 15..8
          G_OEn_HI <= '0';
          B_A0     <= '0';
        else
          -- byte access odd address only or word access second cycle (odd byte)
          -- write LSB 7..0
          -- G_OEn_LO active
          G_OEn_LO <= '0';
          B_A0     <= '1';
        end if;
          
        data_cs_n <= '0';

        s_io_3_w <= '0';
        s_io_4   <= '1';
      end if;

      if s_io_4 = '1' then
        -- Bus data cycle, up to start of periph acknowledge

        if B_ACKn_cdc = '0' then

          casl_n <= rf_cnt(0) or (not dma_in_progress);
          casu_n <= (not rf_cnt(0)) or (not dma_in_progress);

          -- no DTACK# during DMA or during first bus cycle of a word access
          DTACKn_sr <= (others => (dma_in_progress or even_cycle));

          M_WEn <= not dma_in_progress;
          
          -- DMA
          a_sel_hilo <= '0';

          s_io_4 <= '0';
          s_io_5 <= '1';
        end if;
      end if;

      if s_io_5 = '1' then
        if even_cycle = '1' then
          -- one turnover cycle in between even and odd bytes
          G_OEn_HI <= '1';
        end if;
        
        casl_n <= '1';
        casu_n <= '1';

        s_io_5 <= '0';

        if CPU_RWn = '1' then
          s_io_6 <= '1';
        else
          data_cs_n <= '1';

          s_io_7 <= '1';
        end if;
      end if;

      if s_io_6 = '1' then
        -- hold B_CEn and so read data
        data_cs_n <= '1';
        
        s_io_6 <= '0';
        s_io_7 <= '1';
      end if;
        
      if s_io_7 = '1' then
        -- Bus epilogue, up to end of periph acknowledge
        ras_n  <= '1';

        M_WEn  <= ras_n or not dma_in_progress;

        a_sel_address <= '1';
 
        a_sel_hilo <= '1';

        G_OEn    <= '1';
        G_OEn_LO <= '1';
        G_OEn_HI <= '1';
        
        B_A0     <= '1';  -- in case of second (odd byte) sub-access 

        if B_ACKn_cdc /= '0' then
          s_io_7 <= '0';
          
          even_cycle <= '0';

          if even_cycle = '1' then
            -- word write access, end of first cycle (even byte)
            -- start second (odd byte) sub-access
            data_cs_n <= '0';
            G_OEn_LO <= '0';
            a_sel_address <= '0';
            s_io_4   <= '1';
          else
            s_0      <= '1';
          end if;
        end if;
      end if;

      if s_dram = '1' then
        -- DRAM acccess
        a_sel_hilo <= '1';

        -- state - stay 2 cycles in s_dram state
        s_dram <= not a_sel_hilo;
        s_0    <= a_sel_hilo;
      end if;

      if s_ref = '1' then
        -- refresh cycle
        if casl_n = '0' then
          casl_n <= '1';
          casu_n <= '1';
          ras_n  <= '0';
        elsif dly = '0' then
          -- hold RAS# low for 2 cycles
          dly <= '1';
        elsif ras_n = '0' then
          ras_n <= '1';
        else
          s_ref <= '0';
          s_0   <= '1';
        end if;
      end if;

      --------------------------------------------------------------------------
      -- counter for refresh
      --------------------------------------------------------------------------
      if dma_in_progress = '0' or s_io_3_r = '1' then
        rf_cnt <= rf_cnt + 1;

        if dma_in_progress = '1' and rf_cnt(2 downto 0) = "110" then
          -- one refresh every 8 accesses during DMA
          rf_req <= '1';
        end if;
      end if;

      if (not rf_cnt) = 0 and s_io_7 = '1' then
        dma_in_progress <= '0';
      end if;
      
      -- 1024 refresh cycles every 16 ms
      -- at 8MHz, every 125 cycles => rounded up to 128 ...
      if dma_in_progress = '0' and rf_cnt(6 downto 0) = 0 then
        rf_req <= '1';
      end if;

      --------------------------------------------------------------------------
      -- register update
      --------------------------------------------------------------------------
      if reg_update = '1' then
        -- internal PMU registers
        if CPU_A(2 downto 1) = "00" then
          -- base_lo
          reg_pmu_base(19 downto 12) <= CPU_D;
        elsif CPU_A(2 downto 1) = "01" then
          -- base_hi
          reg_pmu_base(22 downto 20) <= CPU_D(10 downto 8);
        elsif CPU_A(2 downto 1) = "10" then
          -- mask_lo
          reg_pmu_mask(19 downto 12) <= CPU_D;
        else
          -- mask_hi - 4MiB
          reg_pmu_mask(21 downto 20) <= CPU_D(9 downto 8);
        end if;
      end if;

    end if;
  end process;

  process (GCLK) is
  begin
    if falling_edge(GCLK) then
      casl_fall_n <= casl_n and not (s_dram and CPU_RWn);
      casu_fall_n <= casu_n and not (s_dram and CPU_RWn);
    end if;
  end process;
 
    
  process (casl_fall_n, casu_fall_n, s_dram, CPU_RWn, CPU_LDSn, CPU_UDSn) is
  begin
      if
        -- DMA or refresh
        (casl_fall_n = '0') or
        -- CPU => DRAM
        (s_dram = '1' and ((CPU_RWn = '0') and (CPU_LDSn = '0')))
      then
        M_CASLn <= '0';
      else
        M_CASLn <= '1';
      end if;
      
      if
        -- DMA or refresh
        (casu_fall_n = '0') or
        -- CPU => DRAM
        (s_dram = '1' and ((CPU_RWn = '0') and (CPU_UDSn = '0')))
      then
        M_CASUn <= '0';
      else
        M_CASUn <= '1';
      end if;

   end process;

  M_RASn <=
    -- refresh
    '0' when ras_n = '0' else
    -- CPU access
    '0' when CPU_ASn = '0' and ((s_0 = '1' and (rf_req = '0' or CPU_RWn = '0')) or (s_dram = '1') or (s_cpu = '1')) else
    '1';

  CPU_BERRn  <= BERRn_mask or CPU_ASn;
  CPU_VPAn   <= VPAn_mask or CPU_ASn;
  CPU_DTACKn <= DTACKn_sr(0);

  ------------------------------------------------------------------------------
  -- 
  ------------------------------------------------------------------------------
  G_CSn <= (GCLK or addr_cs_n) and
           data_cs_n;
  
  ------------------------------------------------------------------------------
  -- CDC for IO bus acknowledge
  ------------------------------------------------------------------------------
   process (GCLK) is
   begin
     if rising_edge(GCLK) then
       B_ACKn_meta <= B_ACKn;
     end if;
   end process;

   process (GCLK) is
   begin
     if falling_edge(GCLK) then
       B_ACKn_cdc <= B_ACKn_meta;
     end if;
   end process;

end architecture rtl;
