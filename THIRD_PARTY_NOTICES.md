# Third-Party Notices

MQT SCPD is licensed under the MIT License (see `LICENSE`). Its build fetches
the libraries below at configure time (`cmake/ExternalDependencies.cmake`) and
compiles them into the C++ libraries and the Python extension module
`pyscpd`. The wheels therefore carry code under the licenses listed here; the
source distribution carries MIT code only. Every library keeps its own license,
which is included in its fetched sources.

| Library | Version | License | Use |
|---|---|---|---|
| [CDT](https://github.com/artem-ogre/CDT) (artem-ogre) | 1.4.5 | MPL-2.0 | Constrained Delaunay triangulation of the capacity check (`capacity_cdt`, `capacity_cdt_world`); header-only, **modified at build time, see below** |
| [FlatBuffers](https://github.com/google/flatbuffers) | 25.12.19 | Apache-2.0 | Serialization of the configuration and the artifacts |
| [nlohmann/json](https://github.com/nlohmann/json) | 3.12.0 | MIT | JSON reading and writing |
| [HiGHS](https://github.com/ERGO-Code/HiGHS) | 1.11.0 | MIT | Linear and mixed-integer programming; compiles in pdqsort (Zlib) |
| [Boost.Polygon](https://github.com/boostorg/polygon) | 1.89.0 | BSL-1.0 | Polygon geometry |
| [GoogleTest](https://github.com/google/googletest) | 1.17.0 | BSD-3-Clause | Tests only; not part of any distributed artifact |

## CDT (Mozilla Public License 2.0)

CDT is Copyright (c) Artem Amirkhanov and is licensed under the Mozilla Public
License, Version 2.0. You can obtain a copy of the license at
<https://mozilla.org/MPL/2.0/>; the fetched sources carry it as `LICENSE`.
The unmodified source code of the version used is available at
<https://github.com/artem-ogre/CDT/releases/tag/1.4.5>.

MQT SCPD modifies one file of CDT when it fetches the sources
(`cmake/PatchCdt.cmake`, run as the `PATCH_COMMAND` of the fetch):

- `CDT/include/CDTUtils.h`: the type `CDT::EdgeUSet`, the set of constraint
  edges of a triangulation, is changed from `std::unordered_set<Edge>` to
  `mqt_scpd_cdt::FlatEdgeSet<Edge, VertInd, noVertex>`, an open-addressing
  hash table in one flat array, and an `#include` of that header is added.
  A triangulation copied for every state of the CDT world then copies one
  array instead of one node per edge.

The modified `CDTUtils.h` remains covered by the MPL-2.0 (section 3.1). Its
source code form is the upstream file with the replacements that
`cmake/PatchCdt.cmake` applies; the script is part of this repository, and
the modified file is written to `<build>/_deps/artem_cdt-src/CDT/include/` by
every build. The header `src/grid/cdt/FlatEdgeSet.h` that the patch copies in
beside CDT's headers is MQT SCPD's own code and is licensed under the MIT
License; it is a separate file and no modification of CDT.

MQT SCPD also builds CDT with its compile-time option
`CDT_ENABLE_CALLBACK_HANDLER`, a documented interface of the library, to be
told where a crossing of two constraints adds a vertex. This is use, not
modification.
