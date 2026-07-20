-- TNS|Code12|TNE
-- RadioMaster TX12 / EdgeTX tool script.
--
-- Copy this file to:
--   /SCRIPTS/TOOLS/code12.lua
--
-- Default physical mapping:
--   B switch middle: enable Code12 edit/send mode
--   A button: previous digit
--   D button: send current 12-digit code
--   Wheel rotate: change selected digit, 1..4
--   Wheel press: next digit
--   RTN: exit this tool
--
-- Model mix setup:
--   CH13  Source MAX   Weight GV1
--   CH14  Source MAX   Weight GV2
--   CH15  Source MAX   Weight GV3
--   CH16  Source MAX   Weight GV4
--
-- Receiver/MCU:
--   CH13-CH15 carry the 12 digits, packed as three 0..255 values.
--   CH16 toggles whenever D or long wheel press sends the code.

local MODE_SWITCH = "sb" -- Physical B switch. Change if your mode switch is not B.
local BTN_A = "sa"       -- Physical A button.
local BTN_D = "sd"       -- Physical D button.

local MODE_MID_LIMIT = 200
local BUTTON_DELTA = 500

local digits = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }
local cursor = 1
local send_count = 0
local trigger = -100
local dirty = true
local ignore_enter_break = false
local button_rest = {}
local button_last = {}
local has_model_api = model and model.setGlobalVariable

local function has(code)
  return code ~= nil
end

local function is_event(event, ...)
  for i = 1, select("#", ...) do
    local code = select(i, ...)
    if has(code) and event == code then
      return true
    end
  end
  return false
end

local function source_value(name)
  if name == nil or name == "" then
    return nil
  end
  return getValue(name)
end

local function remember_button_rest(name)
  if name == nil or name == "" then
    return
  end
  local value = source_value(name)
  if value ~= nil then
    button_rest[name] = value
  end
  button_last[name] = false
end

local function button_pressed(name)
  local value = source_value(name)
  if value == nil then
    return false
  end

  if button_rest[name] == nil then
    button_rest[name] = value
  end

  return math.abs(value - button_rest[name]) > BUTTON_DELTA
end

local function button_rise(name)
  local pressed = button_pressed(name)
  local last = button_last[name] or false
  button_last[name] = pressed
  return pressed and not last
end

local function in_code_mode()
  if MODE_SWITCH == nil or MODE_SWITCH == "" then
    return true
  end

  local value = source_value(MODE_SWITCH)
  if value == nil then
    return true
  end

  return value > -MODE_MID_LIMIT and value < MODE_MID_LIMIT
end

local function pack4(index)
  return (digits[index] - 1) * 64
    + (digits[index + 1] - 1) * 16
    + (digits[index + 2] - 1) * 4
    + (digits[index + 3] - 1)
end

local function scale_pack(value)
  return math.floor((value * 200 + 127) / 255) - 100
end

local function set_gv(index, value)
  if not has_model_api then
    return
  end

  for fm = 0, 8 do
    model.setGlobalVariable(index, fm, value)
  end
end

local function write_outputs()
  set_gv(0, scale_pack(pack4(1)))
  set_gv(1, scale_pack(pack4(5)))
  set_gv(2, scale_pack(pack4(9)))
  set_gv(3, trigger)
  dirty = false
end

local function inc_digit()
  digits[cursor] = digits[cursor] + 1
  if digits[cursor] > 4 then
    digits[cursor] = 1
  end
  dirty = true
end

local function dec_digit()
  digits[cursor] = digits[cursor] - 1
  if digits[cursor] < 1 then
    digits[cursor] = 4
  end
  dirty = true
end

local function next_cursor()
  cursor = cursor + 1
  if cursor > 12 then
    cursor = 1
  end
end

local function prev_cursor()
  cursor = cursor - 1
  if cursor < 1 then
    cursor = 12
  end
end

local function send_code()
  trigger = -trigger
  send_count = send_count + 1
  dirty = true
  write_outputs()
end

local function flags()
  return INVERS or 0, BLINK or 0, SMLSIZE or 0, RIGHT or 0
end

local function draw_digits(y)
  local inverse, blink = flags()
  for i = 1, 12 do
    local attr = 0
    if i == cursor then
      attr = inverse + blink
    end
    lcd.drawText(4 + (i - 1) * 10, y, tostring(digits[i]), attr)
  end
  lcd.drawText(2 + (cursor - 1) * 10, y + 10, "^", 0)
end

local function draw()
  local inverse, _, small, right = flags()
  local p1 = pack4(1)
  local p2 = pack4(5)
  local p3 = pack4(9)

  lcd.clear()
  lcd.drawText(0, 0, "C12", inverse)

  if in_code_mode() then
    lcd.drawText(25, 0, "B:EDIT", inverse)
  else
    lcd.drawText(25, 0, "B:LOCK", inverse)
  end

  lcd.drawText(83, 0, "S", inverse)
  lcd.drawNumber(126, 0, send_count, inverse + right)

  draw_digits(15)

  lcd.drawText(0, 36, "WHEEL VAL  PRESS NEXT", small)
  lcd.drawText(0, 45, "A PREV   D SEND", small)

  if has_model_api then
    lcd.drawText(0, 55, "P", small)
    lcd.drawNumber(14, 55, p1, small)
    lcd.drawNumber(48, 55, p2, small)
    lcd.drawNumber(82, 55, p3, small)
  else
    lcd.drawText(0, 55, "NO MODEL API", inverse)
  end
end

local function handle_physical_buttons()
  if button_rise(BTN_A) then
    prev_cursor()
  end

  if button_rise(BTN_D) then
    send_code()
  end
end

local function handle_event(event)
  if is_event(event, EVT_VIRTUAL_EXIT, EVT_EXIT_BREAK) then
    return 2
  end

  handle_physical_buttons()

  if not in_code_mode() then
    return 0
  end

  if is_event(event, EVT_ENTER_LONG, EVT_ROT_LONG, EVT_VIRTUAL_ENTER_LONG) then
    send_code()
    ignore_enter_break = true
  elseif is_event(event, EVT_VIRTUAL_NEXT, EVT_PLUS_FIRST, EVT_PLUS_REPT, EVT_ROT_RIGHT, EVT_VIRTUAL_INC, EVT_VIRTUAL_INC_REPT) then
    inc_digit()
  elseif is_event(event, EVT_VIRTUAL_PREV, EVT_MINUS_FIRST, EVT_MINUS_REPT, EVT_ROT_LEFT, EVT_VIRTUAL_DEC, EVT_VIRTUAL_DEC_REPT) then
    dec_digit()
  elseif is_event(event, EVT_VIRTUAL_ENTER, EVT_ENTER_BREAK, EVT_ROT_BREAK) then
    if ignore_enter_break then
      ignore_enter_break = false
    else
      next_cursor()
    end
  end

  return 0
end

local function init()
  remember_button_rest(BTN_A)
  remember_button_rest(BTN_D)
  dirty = true
  write_outputs()
end

local function run(event)
  local exit_code = handle_event(event)
  if dirty then
    write_outputs()
  end
  draw()
  return exit_code
end

return { init = init, run = run }
