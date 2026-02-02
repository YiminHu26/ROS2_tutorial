# Git note

## Clone
```bash
$ git clone https://github.com/path/to/original-repo.git

# For repos that I have no permit to push directly
# Fork the repository and add the original repo as upstream
# So that we can request to pull it later

# 1 Go to the original repo and click "folk". It will create a copy under my username
# 2 Clone that copy
$ git clone https://github.com/YOUR-USERNAME/original-repo.git
# 3 Add the original as upstream (ssh preferred, as fetch over http may ask for username and password)
$ git remote add upstream git@gitlab.xxxxxxxx
# 4 Validate
$ git remote -v
# Where "origin" and "upstream" repo will be displayed.
```

## Push (Only for my own project)
```bash
# 1 It is recommended to create a new branch for temporary changes/development
$ git switch -c dev
# 2 Stage files (Working directory -> Staging Area) 
$ git add . # "."means staging all modified files
$ git add /path/to/certain/files # stage certain files/folders
# 3 Commit files (Staging Area -> Local Repository)
$ git commit -m "xxxx" # Commit with a (simple) comment
$ git commit # Without -m, will be prompted to input comment in a seperate file, can be multiple lines
# 4 Push files (Local Repository -> Remote Repository)
$ git push
$ git push origin main # Sometimes it is required
```

## Colaberate
```bash
# Scenario 1: I have forked a repo and made changes to it. Before I can commit those files, there has been changes in the original repo.
# 1 Make sure upstream repo has been set
$ git remote -v
$ git remote add upstream git@gitlab.xxxxxxxx # Otherwise
# 2 Switch to the local main branch
$ git switch main
# 3 Fetch latest changes from upstream (Remote -> Local Repository)
$ git fetch upstream
# 4 Merge upstream into my branch
$ git merge upstream/main
# What happens:
# If no conflicts → merge commit is created
# If conflicts → Git tells you which files
# 5 If conflicts, edit the files and commit and push them again
$ git add <fixed-file>
$ git commit
$ git push # Normally directly to origin/main
$ git push origin dev
# 6 In Github/Gitlab create pull request to merge

# Bonus: Keep feature branches up to date
$ git switch dev

$ git merge main
# or
$ git rebase main
```
