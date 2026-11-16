$ErrorActionPreference = 'Stop'

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$armPath = Resolve-Path (Join-Path $scriptRoot '..\Control\arm_motor.c')
$rtosPath = Resolve-Path (Join-Path $scriptRoot '..\User\freertos_start.c')

$armText = Get-Content -LiteralPath $armPath -Raw
$rtosText = Get-Content -LiteralPath $rtosPath -Raw

function Test-ArmSelectorMapping {
    param(
        [string]$Text,
        [string]$FPos,
        [string]$CPos,
        [string]$TaskName
    )

    $fPattern = 'ARM_PRESET_F_MODE_VALUE == ' + $FPos
    $start = $Text.IndexOf($fPattern)
    if ($start -lt 0) {
        return $false
    }

    $nextF = $Text.IndexOf('ARM_PRESET_F_MODE_VALUE ==', $start + $fPattern.Length)
    $end = $Text.IndexOf('return NULL;', $start)
    if (($nextF -gt 0) -and (($end -lt 0) -or ($nextF -lt $end))) {
        $end = $nextF
    }
    if ($end -lt 0) {
        $end = $Text.Length
    }

    $block = $Text.Substring($start, $end - $start)
    $pattern = '(?s)ARM_PRESET_C_MODE_VALUE\s*==\s*' + [regex]::Escape($CPos) +
               '.*?return\s+&' + [regex]::Escape($TaskName) + '\s*;'
    return ($block -match $pattern)
}

$checks = @(
    @{
        Name = 'Arm path point stores hold time'
        Pass = ($armText -match '(?m)^\s*uint16_t\s+hold_time_ms\s*;')
    },
    @{
        Name = 'F3 C1 maps to pick height 3 task'
        Pass = (Test-ArmSelectorMapping $armText 'ARM_SWITCH_POS_3' 'ARM_SWITCH_POS_1' 'ARM_TASK_PICK_HEIGHT3')
    },
    @{
        Name = 'F3 C2 maps to pick height 2 task'
        Pass = (Test-ArmSelectorMapping $armText 'ARM_SWITCH_POS_3' 'ARM_SWITCH_POS_2' 'ARM_TASK_PICK_HEIGHT2')
    },
    @{
        Name = 'F3 C3 maps to pick height 1 task'
        Pass = (Test-ArmSelectorMapping $armText 'ARM_SWITCH_POS_3' 'ARM_SWITCH_POS_3' 'ARM_TASK_PICK_HEIGHT1')
    },
    @{
        Name = 'F2 C3 selects place task'
        Pass = (Test-ArmSelectorMapping $armText 'ARM_SWITCH_POS_2' 'ARM_SWITCH_POS_3' 'ARM_TASK_PLACE_BLOCK')
    },
    @{
        Name = 'F2 C3 place mode lets A/D directly control suction'
        Pass = (($armText -match 'Arm_Place_Mode_Selected') -and
                ($armText -match 'Arm_Place_Mode_Selected\s*\(\s*\)\s*!=\s*0U') -and
                ($armText -match 'Arm_Suction_Set\s*\(\s*1U\s*\)') -and
                ($armText -match 'Arm_Suction_Set\s*\(\s*0U\s*\)'))
    },
    @{
        Name = 'F2 C2 transfer uses A to open suction and D to move without closing suction'
        Pass = (($armText -match 'ARM_TASK_TRANSFER_PICK') -and
                ($armText -match 'ARM_TASK_TRANSFER_PLACE') -and
                ($armText -match '\{-0\.052f,\s*-2\.845f,\s*0\.236f,\s*ARM_TASK_DEFAULT_HOLD_MS\}') -and
                ($armText -match '\{0\.012f,\s*-2\.415f,\s*-1\.477f,\s*ARM_TASK_DEFAULT_HOLD_MS\}') -and
                ($armText -match 'Arm_Transfer_Mode_Selected') -and
                ($armText -match '(?s)Arm_Transfer_Mode_Selected\s*\(\s*\)\s*!=\s*0U.*?pick_key_pressed\s*!=\s*0U.*?Arm_Start_Path_Task\s*\(\s*&ARM_TASK_TRANSFER_PICK\s*,\s*ARM_TASK_ACTION_OPEN_SUCTION\s*\)') -and
                ($armText -match '(?s)Arm_Transfer_Mode_Selected\s*\(\s*\)\s*!=\s*0U.*?recycle_key_pressed\s*!=\s*0U.*?Arm_Start_Path_Task\s*\(\s*&ARM_TASK_TRANSFER_PLACE\s*,\s*ARM_TASK_ACTION_NONE\s*\)') -and
                ($armText -notmatch 'Arm_Start_Path_Task\s*\(\s*&ARM_TASK_TRANSFER_PLACE\s*,\s*ARM_TASK_ACTION_CLOSE_SUCTION\s*\)'))
    },
    @{
        Name = 'Suction state uses shared Arm_Suction_Set API'
        Pass = (($armText -match 'void\s+Arm_Suction_Set\s*\(\s*uint8_t\s+enable\s*\)') -and
                ($armText -match 'arm_suction_auto_enabled\s*=\s*enable') -and
                ($armText -match '(?s)void\s+Arm_Suction_Set\s*\(\s*uint8_t\s+enable\s*\).*?HAL_GPIO_WritePin\s*\(\s*GPIOE\s*,\s*GPIO_PIN_3\s*,.*?\?\s*GPIO_PIN_SET\s*:\s*GPIO_PIN_RESET') -and
                ($rtosText -match 'Arm_Suction_Set\s*\(') -and
                ($rtosText -notmatch 'HAL_GPIO_WritePin\s*\(\s*GPIOE\s*,\s*GPIO_PIN_3'))
    },
    @{
        Name = 'B1 F2 C3 bypasses QR branch and controls suction before claw'
        Pass = (($rtosText -match 'ch6\s*=\s*remote\.ch\[6\]') -and
                ($rtosText -match '\(ch7\s*==\s*1\)\s*&&\s*\(ch6\s*==\s*2\)\s*&&\s*\(ch8\s*==\s*3\)') -and
                ($rtosText -match 'place_suction_control_selected') -and
                ($rtosText -match 'else\s+if\s*\(\s*\(ch7\s*!=\s*3\)\s*&&\s*\(ch5\s*==\s*3\)'))
    },
    @{
        Name = 'Teach print requires C1 and F1'
        Pass = (($armText -match 'ARM_TEACH_C_VALUE') -and
                ($armText -match 'ARM_TEACH_F_VALUE'))
    },
    @{
        Name = 'Task runner supports multi-point sequences'
        Pass = (($armText -match 'Arm_Path_Task_t') -and
                ($armText -match 'arm_active_task_point_index'))
    },
    @{
        Name = 'Entering arm mode syncs current F+C selection without starting task'
        Pass = (($armText -match 'void\s+Arm_Mode_Enter\s*\(\s*void\s*\)') -and
                ($armText -match 'arm_last_selected_task\s*=\s*Arm_Get_Selected_Switch_Task\s*\(\s*\)') -and
                ($armText -match 'if\s*\(\s*arm_active_task\s*==\s*NULL\s*\)') -and
                ($rtosText -match 'Arm_Mode_Enter\s*\(\s*\)'))
    },
    @{
        Name = 'Auto arm task continues outside arm mode without joystick input'
        Pass = (($armText -match 'void\s+Arm_Auto_Task_Update\s*\(\s*void\s*\)') -and
                ($armText -match 'Arm_Run_Active_Path_Task\s*\(\s*\)') -and
                ($rtosText -match '(?s)else\s*\{(?:\s|/\*.*?\*/)*Arm_Auto_Task_Update\s*\(\s*\)'))
    },
    @{
        Name = 'B3 suction-linked pick starts from A key and opens GPIOE3 immediately'
        Pass = (($armText -match 'ARM_PICK_KEY_VALUE') -and
                ($armText -match 'ARM_TASK_ACTION_OPEN_SUCTION') -and
                ($armText -match '(?s)static\s+void\s+Arm_Start_Path_Task\s*\([^)]*\).*?ARM_TASK_ACTION_OPEN_SUCTION.*?Arm_Suction_Set\s*\(\s*1U\s*\)') -and
                ($armText -notmatch '(?s)static\s+void\s+Arm_Finish_Active_Path_Task\s*\([^)]*\)(?:(?!static\s+void\s+Arm_Start_Path_Task)[\s\S])*ARM_TASK_ACTION_OPEN_SUCTION'))
    },
    @{
        Name = 'B3 recycle starts from D key and closes GPIOE3 at task finish'
        Pass = (($armText -match 'ARM_RECYCLE_KEY_VALUE') -and
                ($armText -match 'ARM_TASK_ACTION_CLOSE_SUCTION') -and
                ($armText -match 'Arm_Suction_Set\s*\(\s*0U\s*\)'))
    },
    @{
        Name = 'B3 A/D automation ignores E switch selection'
        Pass = (($armText -notmatch 'suction_mode_selected') -and
                ($armText -notmatch 'ARM_SUCTION_MODE_VALUE'))
    },
    @{
        Name = 'F3 C123 can start recycle from D key'
        Pass = (($armText -match 'Arm_Recycle_Command_From_Pick_Position') -and
                ($armText -match 'ARM_PRESET_F_MODE_VALUE\s*==\s*ARM_SWITCH_POS_3') -and
                ($armText -match 'selected_recycle_task\s*=\s*Arm_Get_Selected_Recycle_Task\s*\(\s*\)'))
    },
    @{
        Name = 'F3 D recycle is not immediately cancelled until C changes'
        Pass = (($armText -match 'Arm_Get_Recycle_Last_Selected_Task') -and
                ($armText -match 'arm_last_selected_task\s*=\s*Arm_Get_Recycle_Last_Selected_Task\s*\(\s*\)'))
    },
    @{
        Name = 'F3 D recycle keeps C height return available after suction closes'
        Pass = (($armText -match 'arm_height_return_enabled') -and
                ($armText -match 'arm_height_return_enabled\s*=\s*1U') -and
                ($armText -match 'arm_suction_auto_enabled\s*!=\s*0U\)\s*\|\|') -and
                ($armText -match 'arm_height_return_enabled\s*!=\s*0U'))
    },
    @{
        Name = 'B3 GPIO task does not directly handle A/D suction control'
        Pass = (($rtosText -match 'ch7\s*!=\s*3') -and
                ($rtosText -notmatch 'arm_teach_print_pressed'))
    },
    @{
        Name = 'Old B3 C1 blanket GPIO block is removed'
        Pass = ($rtosText -notmatch 'else\s+if\s*\(\s*\(ch7\s*==\s*3\)\s*&&\s*\(ch8\s*==\s*1\)\s*\)')
    }
)

$failed = 0
foreach ($check in $checks) {
    if ($check.Pass) {
        Write-Host "[PASS] $($check.Name)"
    } else {
        Write-Host "[FAIL] $($check.Name)"
        $failed++
    }
}

if ($failed -ne 0) {
    exit 1
}
