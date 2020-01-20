Git Workflow - Draft
============

## Intro

Of course there is no One True Way™ of working with code across teams, and no document should keep you from improving things or doing something you using your best judgment find necessary.

![Image description](git_workflow_img.jpg)

## The Master Branch

The **Master Branch** should be considered sacred, the code should always be working and be ready to use. 

You should never work directly in master and never push to it from a local version or from any branch other than the **development-branch**. 

Generally nothing should happen in the the **Master branch** unless there is a new release.


## Development Branch

The **development branch** is from where everyone branches off and merges in most of their new code. 

As with the master branch it should always be working, but not necessarily be ready for release. 

Try to never write code directly to this branch. Make a pull requests from the relevant feature branch  and assign one or two developers to review the code. 

## Feature Branches

When working on a new feature a **feature branches** should be created from the development branch.

Try to make pull requests from a working branch and assign one or two developers to review the code. 
Feature branches should be named: `feature_[branch_name]`

## Working branches

These are branches created from a feature branch where a developer can work freely.
Working branches should be named: `[initials]_[branch_name]`


## New Code

### Create a New Branch

1. Update the **master**, **dev** and/or **feature** branches to latest version(s)
2. Create branch
    - Always create a new branch from the place it will be merged back into
    - names should be kept short and descriptive. 
    - prefixed with initials (eg. `JB_branch_name`)

### Pushing and Commiting Changes

3. Make changes
    - Make sure everything is running (build, tests, etc)
4. Pull branch for any changes (If not working alone on specific branch)
5. Commit with descriptive name
    - prefix with `[WIP]` ("work in progress") if anything is broken.
6. Push branch up to github

### Pull Request

7. Open pull request into relevant branch
    - Write a descriptive title of task
    - Bullet points with descriptions of changes
    - assign one or two people to review changes. 

### Code Review

8. Other devs should review code and try to leave constructive notes if necessary
    - check for potential issues
    - if confusing to you, probably confusing to someone else
    - general improvements

9. Code owner is responsible for making any necessary fixes any pushing them up
    - Follow same commit process
    
10. Other devs should give the code the thumbs up (at least two other devs)
    - add short text of why you are okay with merging the pull request

### Merging Code

Once you have had enough people review it, you are confident in the code you have written, and everything is well tested and passing. You can start the deploy process.

11. Merge the pull request into the **staging** or **feature** branch, and then delete it on github.
12. Run applicable test again on the merged code.

### Add changes to release Draft
Go to [releases](https://github.com/RI-SE/Maestro/releases) and add your pull request to the current release draft. Also remember to add the change to the changelog.md file in Maestro/Maestro-Tools. 

If the change warrants it increase the version numer. 


#

## Additional stuff and resources

- [A Successful Git Branching Model](http://nvie.com/posts/a-successful-git-branching-model/)
- [Understanding the Git Workflow](https://sandofsky.com/blog/git-workflow.html)
- [Github: Understanding the Git Flow](http://guides.github.com/overviews/flow/)
- [Issues with Git Flow](http://scottchacon.com/2011/08/31/github-flow.html)
