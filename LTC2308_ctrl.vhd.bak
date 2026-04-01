-- LTC2308_ctrl.vhd
-- 
-- This module implements an SPI controller customized for an LTC2308 
-- Analog-to-Digital Converter (ADC). 
--
-- Generics:
--   CLK_DIV : Divides the main clock to generate the SCLK frequency.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity LTC2308_ctrl is
	generic (
		-- Divides the main clock to generate SCLK frequency
		-- Note that there's an additional factor of 2 because
		-- this CLK_DIV factor defines the rate at which SCLK
		-- will rise *and* fall.
		CLK_DIV : integer := 1 
	);
	port (
		-- Control and data for this device
		clk      : in  std_logic;
		nrst     : in  std_logic;
		start    : in  std_logic;
		rx_data  : out std_logic_vector(11 downto 0);
		busy     : out std_logic;
		
		-- SPI Physical Interface
		sclk     : out std_logic; -- Serial clock
		conv     : out std_logic; -- Conversion start control
		mosi     : out std_logic; -- Data out from this device, in to ADC
		miso     : in  std_logic  -- Data out from ADC, in to this device
	);
end entity LTC2308_ctrl;

architecture internals of LTC2308_ctrl is

	-- Expanded state machine to handle conversion wait times
	type state_type is (IDLE, CONV_PULSE, CONV_WAIT, TRANSFER, HOLD);
	signal state : state_type;
	
	-- Internal signals for clock generation
	signal clk_cnt   : integer range 0 to CLK_DIV;
	signal sclk_int  : std_logic;
	signal sclk_rise : std_logic;
	signal sclk_fall : std_logic;
	
	-- Internal signals for command/control
	signal bit_cnt   : integer range 0 to 12;
	signal wait_cnt  : integer range 0 to 200; -- New counter for 200 clk wait
	
	-- Internal signals for data shifting
	-- The default value here is for a single-ended conversion on channel 0
	constant tx_data : std_logic_vector(11 downto 0) := "100010000000";
	signal tx_reg    : std_logic_vector(11 downto 0);
	signal rx_reg    : std_logic_vector(11 downto 0);

begin

	-- Output assignment for the SPI Clock.
	-- An internal version is needed so that logic inside this device can be based on
	-- it (reading an output is not allowed in VHDL).
	sclk <= sclk_int;

	-------------------------------------------------------------------
	-- Controlling Process
	-- Handles the start signal, busy flag, wait timer, 
	-- and counts the 12 bits as they are transmitted.
	-------------------------------------------------------------------
	process(clk, nrst)
	begin
		if nrst = '0' then
			state    <= IDLE;
			bit_cnt  <= 0;
			wait_cnt <= 0;
			conv     <= '0';
			busy     <= '0';
		elsif rising_edge(clk) then
			case state is
				when IDLE =>
					conv <= '0';
					busy <= '0';
					if start = '1' then
						state   <= CONV_PULSE;
						conv    <= '1'; -- Go high for one clk cycle
						busy    <= '1';
					end if;

				when CONV_PULSE =>
					conv     <= '0'; -- Go low to keep ADC awake
					wait_cnt <= 40;  -- Set wait timer
					state    <= CONV_WAIT;

				when CONV_WAIT =>
					if wait_cnt = 0 then
						state   <= TRANSFER;
						bit_cnt <= 12 - 1;
					else
						wait_cnt <= wait_cnt - 1;
					end if;

				when TRANSFER =>
					-- Decrement bit count on the rising edge
					if sclk_rise = '1' then
						bit_cnt <= bit_cnt - 1;
						if bit_cnt = 0 then
							state <= HOLD;
						end if;
					end if;
						  
				when HOLD =>
					conv <= '0';
					busy <= '0';
					-- Only allow retrigger if start gets deasserted
					if start = '0' then
						state   <= IDLE;
					end if;
						  
			end case;
		end if;
	end process;

	-------------------------------------------------------------------
	-- Clock Generation Process
	-- Divides the system clock for SCLK and generates flag signals to 
	-- control other parts of the system.
	-------------------------------------------------------------------
	process(clk, nrst)
	begin
		if nrst = '0' then
			clk_cnt   <= 0;
			sclk_int  <= '0';
			sclk_rise <= '0';
			sclk_fall <= '0';
		elsif rising_edge(clk) then
			-- Note that because this is in a process, these values
			-- can be "overridden" by lines of code lower in the block.
			sclk_rise <= '0';
			sclk_fall <= '0';
			
			if state = TRANSFER then
				clk_cnt <= clk_cnt + 1;
				if clk_cnt = CLK_DIV - 1 then
					clk_cnt <= 0;
					
					sclk_int <= not sclk_int; -- Toggle SCLK
					if sclk_int = '0' then
						sclk_rise <= '1'; -- SCLK is transitioning 0 -> 1
					else
						sclk_fall <= '1'; -- SCLK is transitioning 1 -> 0
					-- If those IF conditions seem backwards to you, you're
					-- thinking like software instead of thinking like hardware.
					
					end if;
				end if;
			else
				clk_cnt  <= 0;
				sclk_int <= '0'; -- Ensure SCLK idles low
			end if;
		end if;
	end process;

	-------------------------------------------------------------------
	-- Data Process
	-- Manages the TX and RX shift registers.
	-- Samples MISO on rising edges and shifts MOSI out on falling edges.
	-------------------------------------------------------------------
	process(clk, nrst)
	begin
		if nrst = '0' then
			tx_reg  <= (others => '0');
			rx_reg  <= (others => '0');
			rx_data <= (others => '0');
			mosi    <= '0';
		elsif rising_edge(clk) then
			
			if state = IDLE then
				-- Load data to transmit immediately upon start signal
				if start = '1' then
					tx_reg <= tx_data;
					mosi   <= tx_data(11); -- Setup the first bit on MOSI
				end if;
					 
			elsif state = TRANSFER then
				-- Sample MISO on rising edges
				if sclk_rise = '1' then
					rx_reg <= rx_reg(10 downto 0) & miso;
				end if;
					 
				-- Shift MOSI on falling edges.
				if sclk_fall = '1' then
					tx_reg <= tx_reg(10 downto 0) & '0';
					mosi   <= tx_reg(10); -- Put the next MSB onto the line
				end if;
					 
			elsif state = HOLD then
				-- Once the last bit is shifted, latch received data onto
				-- the output bus.
				if sclk_fall = '1' then
					rx_data <= rx_reg;
				end if;
			end if;
			
		end if;
	end process;

end architecture internals;