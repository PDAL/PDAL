(integration)=

# Continuous Integration

PDAL {ref}`regression tests <pdal_test>` are run on a per-commit basis using
[GitHub Actions]

## Status

{{ alpinestatus }}  {{ linuxstatus }}  {{ osxstatus }}  {{ windowstatus }}  {{ docsstatus }}  {{ dockerstatus }}

## Configuration

Continuous integration configuration is modified by manipulating configuration
files in to locations:

- `./github/workflows`
- `./scripts/ci`

Linux, OSX, and Windows builds are all configured separately with scripts in the
`./scripts/ci` directory.

## Dependencies

All of the tests use Conda Forge for dependencies.

The Linux builder has a "fixed" configuration that pins GDAL to a specific
version to prevent the rest of the dependency tree from floating according to
Conda Forge's package dependency rules.

## Docs

Docs are always built and doc artifacts are attached to the build:

- HTML
- PDF
- Misspelled words

### Push to pdal.org

Docs are pushed to pdal.org under the following conditions:

- Doc building succeeds
- The push branch denoted in `./github/workflows/docs.yaml` matches the current
  `*-maintenance` branch.

[github actions]: https://github.com/features/actions
