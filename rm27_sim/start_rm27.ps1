[CmdletBinding()]
param(
    [switch]$Check,
    [switch]$ServerOnly,
    [ValidateRange(0, 1000000)][int]$Iterations = 0
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$modelRoot = Join-Path $projectRoot 'models'
$modelPath = Join-Path $modelRoot 'rmuc_2027/model.sdf'
$configPath = Join-Path $modelRoot 'rmuc_2027/model.config'
$meshPath = Join-Path $modelRoot 'rmuc_2027/meshes/rmuc_2027.stl'
$worldPath = Join-Path $projectRoot 'worlds/rm27_world.sdf'
$guiPath = Join-Path $projectRoot 'worlds/rm27_gui.config'

foreach ($requiredFile in @($modelPath, $configPath, $meshPath, $worldPath, $guiPath)) {
    if (-not (Test-Path -LiteralPath $requiredFile -PathType Leaf)) {
        throw "Required file not found: $requiredFile"
    }
}
if ($Iterations -gt 0 -and -not $ServerOnly) {
    throw '-Iterations requires -ServerOnly.'
}

# Validate XML before starting a GUI; gz sdf --check can report errors with exit 0.
$modelXml = [xml]([IO.File]::ReadAllText($modelPath))
$worldXml = [xml]([IO.File]::ReadAllText($worldPath))
$configXml = [xml]([IO.File]::ReadAllText($configPath))
$arenaModel = $worldXml.sdf.world.model | Where-Object { $_.name -eq 'arena_textured' }
if (-not $arenaModel) {
    throw 'rm27_world.sdf has no arena_textured model; the arena would render untextured.'
}

$previousResourcePath = $env:GZ_SIM_RESOURCE_PATH
try {
    # Process-local only: no machine/user environment or Pixi dependency changes.
    $resourcePaths = @($modelRoot)
    if ($previousResourcePath) {
        $resourcePaths += $previousResourcePath -split [regex]::Escape([IO.Path]::PathSeparator)
    }
    $env:GZ_SIM_RESOURCE_PATH = ($resourcePaths | Select-Object -Unique) -join [IO.Path]::PathSeparator
    Push-Location -LiteralPath $PSScriptRoot
    try {
        if ($Check) {
            $validation = @(& pixi run --locked gz sdf --check $modelPath 2>&1)
            $validationText = ($validation | ForEach-Object { $_.ToString() }) -join "`n"
            Write-Output $validationText
            if ($LASTEXITCODE -ne 0 -or $validationText -notmatch '(?m)^Valid\.\s*$') {
                throw 'Gazebo SDFormat model validation failed.'
            }
            Write-Output "World XML: $($worldXml.sdf.world.name)"
            Write-Output "Resource path: $env:GZ_SIM_RESOURCE_PATH"
            Write-Output "Mesh scale: $($modelXml.sdf.model.link.visual.geometry.mesh.scale)"
            Write-Output "Arena mesh: $($arenaModel.link.visual.geometry.mesh.uri)"
            Write-Output "Original STL SHA256: $((Get-FileHash -LiteralPath $meshPath -Algorithm SHA256).Hash)"
            Write-Output 'Static checks passed. Use a normal launch to verify rendering.'
        }
        else {
            $launchArguments = @('run', '--locked', 'gz', 'sim', '-v', '4')
            if ($ServerOnly) {
                $launchArguments += '-s'
                if ($Iterations -gt 0) {
                    $launchArguments += @('-r', '--iterations', [string]$Iterations)
                }
            }
            else {
                $launchArguments += @('--gui-config', $guiPath)
            }
            $launchArguments += $worldPath
            Write-Output "World: $worldPath"
            Write-Output "Resource path: $env:GZ_SIM_RESOURCE_PATH"
            & pixi @launchArguments
            if ($LASTEXITCODE -ne 0) {
                throw "Gazebo exited with code $LASTEXITCODE."
            }
        }
    }
    finally {
        Pop-Location
    }
}
finally {
    $env:GZ_SIM_RESOURCE_PATH = $previousResourcePath
}
