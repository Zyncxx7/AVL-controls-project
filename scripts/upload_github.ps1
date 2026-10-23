param(
    [string]$RepositoryName = 'AVL-controls-project',
    [switch]$CheckOnly
)

$ErrorActionPreference = 'Stop'
$projectRoot = if ($PSScriptRoot) { Split-Path -Parent $PSScriptRoot } else { (Get-Location).Path }
$projectRoot = (Resolve-Path -LiteralPath $projectRoot).Path
if (-not (Test-Path -LiteralPath (Join-Path $projectRoot '02_c172_live_avl'))) {
    throw 'Run scripts/upload_github.ps1 from the AVL controls project.'
}
$gitSafeDirectory = $projectRoot.Replace('\', '/')
Set-Location -LiteralPath $projectRoot

function Invoke-Git {
    & git -c "safe.directory=$gitSafeDirectory" -C $projectRoot @args
    if ($LASTEXITCODE -ne 0) { throw 'Git command failed.' }
}

if (-not (Get-Command git -ErrorAction SilentlyContinue)) { throw 'Git is not installed.' }
if ($CheckOnly) {
    if (Test-Path -LiteralPath (Join-Path $projectRoot '.git')) {
        Invoke-Git rev-parse --show-toplevel
    }
    Write-Host 'Project location and Git ownership check passed.'
    return
}

# Use an installed CLI, or download GitHub's official portable release.
$cli = Get-Command gh -ErrorAction SilentlyContinue
$portableCli = Get-ChildItem -LiteralPath (Join-Path $projectRoot '.tools\github-cli') -Filter gh.exe -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
if ($cli) {
    $githubCli = $cli.Source
} elseif ($portableCli) {
    $githubCli = $portableCli.FullName
} else {
    $toolFolder = Join-Path $projectRoot '.tools'
    New-Item -ItemType Directory -Path $toolFolder -Force | Out-Null
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    $release = Invoke-RestMethod 'https://api.github.com/repos/cli/cli/releases/latest'
    $asset = $release.assets | Where-Object name -Like '*windows_amd64.zip' | Select-Object -First 1
    $checksums = $release.assets | Where-Object name -Like '*checksums.txt' | Select-Object -First 1
    if (-not $asset -or -not $checksums) { throw 'Official GitHub CLI release assets were not found.' }
    $archive = Join-Path $toolFolder $asset.name
    $checksumFile = Join-Path $toolFolder $checksums.name
    Invoke-WebRequest $asset.browser_download_url -OutFile $archive
    Invoke-WebRequest $checksums.browser_download_url -OutFile $checksumFile
    $entry = Get-Content -LiteralPath $checksumFile | Where-Object { ($_ -split '\s+')[-1] -eq $asset.name }
    if (-not $entry) { throw 'CLI checksum entry was not found.' }
    $expectedHash = ($entry -split '\s+')[0]
    if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne $expectedHash) {
        throw 'GitHub CLI download checksum mismatch.'
    }
    Expand-Archive -LiteralPath $archive -DestinationPath (Join-Path $toolFolder 'github-cli') -Force
    $githubCli = (Get-ChildItem -LiteralPath (Join-Path $toolFolder 'github-cli') -Filter gh.exe -Recurse | Select-Object -First 1).FullName
    if (-not $githubCli) { throw 'GitHub CLI executable was not found.' }
}

& $githubCli auth status --hostname github.com
if ($LASTEXITCODE -ne 0) {
    & $githubCli auth login --hostname github.com --git-protocol https
    if ($LASTEXITCODE -ne 0) { throw 'GitHub sign-in failed.' }
}
& $githubCli auth setup-git --hostname github.com
if ($LASTEXITCODE -ne 0) { throw 'GitHub Git authentication setup failed.' }

$accountJson = & $githubCli api user
if ($LASTEXITCODE -ne 0) { throw 'Unable to read the signed-in GitHub account.' }
$account = $accountJson | ConvertFrom-Json
$repository = "$($account.login)/$RepositoryName"

if (-not (Test-Path -LiteralPath (Join-Path $projectRoot '.git'))) { Invoke-Git init -b main }
$authorName = & git -c "safe.directory=$gitSafeDirectory" -C $projectRoot config --get user.name
if (-not $authorName) {
    $authorName = if ($account.name) { $account.name } else { $account.login }
    Invoke-Git config user.name $authorName
}
$authorEmail = & git -c "safe.directory=$gitSafeDirectory" -C $projectRoot config --get user.email
if (-not $authorEmail) {
    Invoke-Git config user.email "$($account.id)+$($account.login)@users.noreply.github.com"
}
Invoke-Git add --all
& git -c "safe.directory=$gitSafeDirectory" -C $projectRoot diff --cached --quiet
if ($LASTEXITCODE -eq 1) {
    Invoke-Git commit -m 'Organize AVL controls project and verified standalone solvers'
} elseif ($LASTEXITCODE -ne 0) { throw 'Unable to inspect staged changes.' }

$remotes = @(Invoke-Git remote)
if ($remotes -contains 'origin') {
    $origin = Invoke-Git remote get-url origin
    if ($origin -notin @("https://github.com/$repository.git", "https://github.com/$repository", "git@github.com:$repository.git")) {
        throw "The existing origin points elsewhere: $origin"
    }
} else {
    # Create the remote separately, so all local Git commands use the same
    # explicit ownership check, including when the folder was created by Codex.
    & $githubCli repo view $repository --json nameWithOwner
    if ($LASTEXITCODE -ne 0) {
        & $githubCli repo create $repository --private --description 'AVL controls project: C172 6DOF model and standalone C172 and NACA 0012 AVL solvers.'
        if ($LASTEXITCODE -ne 0) { throw 'Repository creation failed.' }
    }
    Invoke-Git remote add origin "https://github.com/$repository.git"
}
Invoke-Git push -u origin main
$localCommit = Invoke-Git rev-parse HEAD
$remoteLine = Invoke-Git ls-remote origin refs/heads/main
if (-not $remoteLine -or ($remoteLine -split '\s+')[0] -ne $localCommit) {
    throw 'Upload could not be verified against the remote main branch.'
}
Write-Host "Uploaded: https://github.com/$repository"
