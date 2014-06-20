# How to contribute

PDAL welcomes all contributions.
We use Github's [pull requests](https://help.github.com/articles/using-pull-requests) to accept patches from the community.

## Getting Started

* Make sure you have a [GitHub account](https://github.com/signup/free).
* Fork the repository on GitHub.

## Making Changes

* Create a topic branch from where you want to base your work.
  * You usually should base your topic branch off of the master branch.
  * To quickly create a topic branch: `git checkout -b my-topic-branch`
* Make commits of logical units.
* Check for unnecessary whitespace with `git diff --check` before committing.
* Make sure your commit messages are in the [proper format](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html).
* Make sure you have added the necessary tests for your changes.
* [Run _all_ the tests](http://www.pdal.io/development/testing.html) to assure nothing else was accidentally broken.

## Submitting Changes

* Push your changes to a topic branch in your fork of the repository.
* Submit a pull request to the repository in the PDAL organization.
  * If your pull request fixes/references an issue, include that issue number in the pull request. For example:

```
Wiz the bang

Fixes #123.
```

* PDAL developers will look at your patch and take an appropriate action.

## Additional Resources

* [PDAL's coding conventions](http://www.pdal.io/development/conventions.html)
* [General GitHub documentation](http://help.github.com/)
* [GitHub pull request documentation](http://help.github.com/send-pull-requests/)
* #pdal IRC channel on freenode.org

## Acknowledgements

The basic skeleton of this CONTRIBUTING file was lifted directly from [Puppet's](https://github.com/puppetlabs/puppet/blob/master/CONTRIBUTING.md).
