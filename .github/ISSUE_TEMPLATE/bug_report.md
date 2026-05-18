---
name: Bug report
about: Create a report to help us improve
title: ''
labels: ''
assignees: ''

---

<!--
Please note that GitHub issues are meant to report bugs or feature requests, and are not intended for "How to" questions or general support. GitHub issues requesting such support will be closed. Luckily, we have an active community on the GIS StackExchange [1] and the PDAL Mailing List [2] that are more than happy to chime in with advice in these situations!

Also note, that issues installing PDAL via Conda should be directed to https://github.com/conda-forge/pdal-feedstock/issues/new. Similarly, issues with the PDAL Python bindings installed via Conda should be directed to https://github.com/conda-forge/python-pdal-feedstock/issues/new.

[1] https://gis.stackexchange.com/questions/tagged/pdal
[2] https://pdal.org/community.html#mailing-list
-->

**Describe the bug**
A clear and concise description of what the bug is, including steps to reproduce the behavior. This typically means providing the PDAL commands you are using (e.g., `pdal translate ...`) and if applicable the contents of any PDAL pipelines you are running.

```
$ pdal translate in.las out.las
```

<details>
  
```
{
  "pipeline":[
    "in.las",
    "out.las"
  ]
}
```

</details>

**Expected behavior**
A clear and concise description of what you expected to happen.

**System/installation information:**
Please provide information on your PDAL version (`pdal --version`) and system (e.g., `uname -a` or `ver`).

```
$ uname -a
```

```
$ pdal --version
```

If installed via Conda, you may be asked to paste the output of `conda list` and `conda info` as well.

<details>
  
```
$ conda list
```

</details>

<details>
  
```
$ conda info
```

</details>

