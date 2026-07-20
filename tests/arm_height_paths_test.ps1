$ErrorActionPreference = 'Stop'

$sourcePath = Join-Path $PSScriptRoot '..\Control\arm_motor.c'
$sourcePath = (Resolve-Path -LiteralPath $sourcePath).Path
$source = Get-Content -LiteralPath $sourcePath -Raw -Encoding utf8

function Get-PathPoints([string]$arrayName)
{
    $arrayPattern = "static\s+const\s+Arm_Path_Point_t\s+$arrayName\[\]\s*=\s*\{(?<body>.*?)\};"
    $arrayMatch = [regex]::Match($source, $arrayPattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)

    if (-not $arrayMatch.Success)
    {
        throw "Path array not found: $arrayName"
    }

    return @([regex]::Matches($arrayMatch.Groups['body'].Value, '\{[^{}]+\}') | ForEach-Object {
        [regex]::Replace($_.Value, '\s+', '')
    })
}

$commonPoint = '{-0.095f,-2.020f,0.145f,ARM_TASK_DEFAULT_HOLD_MS}'
$expectedPaths = @{
    arm_pick_height1_points = @($commonPoint, '{-0.038f,-2.964f,0.170f,ARM_TASK_DEFAULT_HOLD_MS}')
    arm_pick_height2_points = @($commonPoint, '{-0.062f,-2.494f,0.221f,ARM_TASK_DEFAULT_HOLD_MS}')
    arm_pick_height3_points = @($commonPoint, $commonPoint)
}

foreach ($entry in $expectedPaths.GetEnumerator())
{
    $actual = @(Get-PathPoints $entry.Key)
    $expected = @($entry.Value)

    if ($actual.Count -ne 2)
    {
        throw "$($entry.Key) expected 2 points, found $($actual.Count)"
    }

    for ($index = 0; $index -lt 2; $index++)
    {
        if ($actual[$index] -ne $expected[$index])
        {
            throw "$($entry.Key) point $($index + 1) mismatch: $($actual[$index])"
        }
    }
}

Write-Output 'arm height path tests passed'
