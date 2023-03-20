git fetch --tags *>$null

$currentBranch = git branch --show-current
if ($currentBranch -ne 'master')
{
    Write-Host "You are not on the master branch.  Please switch to the master branch by running `git checkout master` before continuing."
    exit
}

$behindCount = git rev-list --count "HEAD..@{u}"
$aheadCount = git rev-list --count "@{u}..HEAD"

if ($behindCount -eq 0 -and $aheadCount -gt 0) 
{
    Write-Host "Your branch is currently ahead of the upstream by $aheadCount commits.  Please merge your changes into master via a pull request before continuing."
    exit
}

if ($behindCount -gt 0 -and $aheadCount -gt 0) 
{
    Write-Host "Your branch and its upstream have diverged.  Please merge your changes into master via a pull request before continuing."
    exit
}

if ($behindCount -gt 0 -and $aheadCount -eq 0)
{
    $confirmation = Read-Host "Your branch is behind the upstream by $behindCount commits.  Do you wish to continue? [y/N]"
    if ($confirmation -ne 'y')
    {
        exit
    }
}

$commit = git rev-parse HEAD
$logMsg = git log --format=%B -n 1 $commit
$date = Get-Date -Format "yyyyMMdd"
$base = "rc"
$tags = git tag -l "$base-$date.*"

$maxTagNum = 0
foreach ($tag in $tags)
{
    if ($tag -match "$base-$date.(\d+)")
    {
        $tagNum = [int]$Matches.1
        if ($tagNum -gt $maxTagNum) 
        {
            $maxTagNum = $tagNum
        }
    }
}

$nextTagNum = $maxTagNum + 1
$tagName = "$base-$date.$nextTagNum"
Write-Host "RC Tag to create: $tagName"
Write-Host "Commit: $commit`n"
Write-Host $logMsg `n

$confirmation = Read-Host "Do you wish to continue? [y/N]"
if ($confirmation -eq 'y') 
{
    git tag -a $tagName -m "Tagging RC version '$tagName'."
    Write-Host "$tagName created."
    git push origin $tagName
    Write-Host "Pushed $tagName to origin."
}
