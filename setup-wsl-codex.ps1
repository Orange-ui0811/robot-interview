[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$distribution = 'Ubuntu-24.04'
$proxyUrl = 'http://127.0.0.1:7890'
$installerUrl = 'https://chatgpt.com/codex/install.sh'

function Set-WslMirroredMode {
    param([AllowEmptyString()][string]$ConfigText)

    $newline = if ($ConfigText.Contains("`r`n") -or -not $ConfigText.Contains("`n")) {
        "`r`n"
    } else {
        "`n"
    }
    $sectionPattern = '(?im)(?<header>^[ \t]*\[wsl2\][ \t]*(?:[;#][^\r\n]*)?(?:\r?\n|$))(?<body>[\s\S]*?)(?=^[ \t]*\[[^\r\n\]]+\]|\z)'
    $section = [regex]::Match($ConfigText, $sectionPattern)
    if (-not $section.Success) {
        $separator = if ($ConfigText.Length -gt 0 -and -not $ConfigText.EndsWith("`n")) {
            $newline
        } else {
            ''
        }
        return $ConfigText + $separator + '[wsl2]' + $newline + 'networkingMode=mirrored' + $newline
    }

    $header = $section.Groups['header'].Value
    if (-not $header.EndsWith("`n")) {
        $header += $newline
    }
    $body = $section.Groups['body'].Value
    $keyPattern = '(?im)^(?<key>[ \t]*networkingMode[ \t]*=[ \t]*)[^;#\r\n]*?(?<suffix>[ \t]*(?:[;#][^\r\n]*)?)(?=\r?$)'
    if ([regex]::IsMatch($body, $keyPattern)) {
        $body = [regex]::Replace($body, $keyPattern, '${key}mirrored${suffix}')
    } else {
        $body = 'networkingMode=mirrored' + $newline + $body
    }

    return $ConfigText.Substring(0, $section.Index) + $header + $body +
        $ConfigText.Substring($section.Index + $section.Length)
}

function Test-WslProxy {
    & wsl.exe -d $distribution --exec curl -fLsS --proxy $proxyUrl --noproxy localhost,127.0.0.1,::1 --connect-timeout 5 --max-time 15 --output /dev/null $installerUrl
    return $LASTEXITCODE -eq 0
}

Write-Host "Checking access to $distribution..."
& wsl.exe -d $distribution --exec /bin/true
if ($LASTEXITCODE -ne 0) {
    throw 'This PowerShell session cannot enter WSL. Run this file in your own Windows PowerShell.'
}

Write-Host 'Checking the Windows proxy...'
& curl.exe -fLsS --proxy $proxyUrl --noproxy localhost,127.0.0.1,::1 --connect-timeout 8 --max-time 20 --output NUL $installerUrl
if ($LASTEXITCODE -ne 0) {
    throw 'The Windows proxy could not download the official Codex installer.'
}

Write-Host 'Checking the proxy from Ubuntu...'
if (-not (Test-WslProxy)) {
    $wslConfig = Join-Path ([Environment]::GetFolderPath('UserProfile')) '.wslconfig'
    $exists = [IO.File]::Exists($wslConfig)
    $current = if ($exists) { [IO.File]::ReadAllText($wslConfig) } else { '' }
    $updated = Set-WslMirroredMode -ConfigText $current
    if ($updated -cne $current) {
        if ($exists) {
            $backup = "$wslConfig.go1-backup-$([DateTime]::Now.ToString('yyyyMMdd-HHmmssfff'))"
            Copy-Item -LiteralPath $wslConfig -Destination $backup
            Write-Host "Existing WSL configuration backed up to $backup"
        }
        [IO.File]::WriteAllText($wslConfig, $updated, [Text.UTF8Encoding]::new($false))
        Write-Host 'Configured networkingMode=mirrored.'
    }

    Write-Host 'Restarting WSL. Running WSL distributions will stop.'
    & wsl.exe --shutdown
    if ($LASTEXITCODE -ne 0) {
        throw 'WSL shutdown failed.'
    }
    if (-not (Test-WslProxy)) {
        throw 'Ubuntu still cannot reach the official installer through the Windows proxy after WSL restart.'
    }
}
Write-Host 'Windows and Ubuntu proxy checks passed.'

$installInUbuntu = @'
set -eu
proxy=$1
export http_proxy="$proxy" https_proxy="$proxy"
export HTTP_PROXY="$proxy" HTTPS_PROXY="$proxy"
export NO_PROXY=localhost,127.0.0.1,::1
export no_proxy="$NO_PROXY"
export PATH="$HOME/.local/bin:$PATH"

if [ -x "$HOME/.local/bin/codex" ] && "$HOME/.local/bin/codex" --version; then
  exit 0
fi
installer="$(mktemp)"
trap 'rm -f -- "$installer"' EXIT
curl -fL --connect-timeout 8 --max-time 30 https://chatgpt.com/codex/install.sh -o "$installer"
CODEX_NON_INTERACTIVE=true sh "$installer" </dev/null
"$HOME/.local/bin/codex" --version
'@

# Encode the static script to pass it intact through Windows' native argument handling.
$installInUbuntu = ($installInUbuntu.Replace("`r`n", "`n") + "`n")
$encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($installInUbuntu))
& wsl.exe -d $distribution --exec /bin/bash -c "set -o pipefail; printf %s $encoded | base64 --decode | /bin/bash -s -- $proxyUrl"
if ($LASTEXITCODE -ne 0) {
    throw 'Codex CLI installation or version verification failed in Ubuntu.'
}
Write-Host 'Codex CLI verified in Ubuntu-24.04.'
