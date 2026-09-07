# Working in ELK

This file applies to this repository. Read any more specific `AGENTS.md` before editing its directory.

## Repository boundaries

- This is the Java Eclipse Layout Kernel: graph models, layout engine, algorithm implementations, and metadata. The sibling `../elkjs` compiles selected Java sources into JavaScript through GWT.
- Put shared layout behavior in Java. Update the JS bridge or packaging in `../elkjs` when needed; do not duplicate an algorithm in JavaScript.
- Start by checking the working tree and HEAD in each affected repository. Preserve unrelated user changes and local agent/index files.
- Use matching ELK/elkjs revisions for cross-repository work. Record both revisions when reporting a rebuilt bundle; package versions alone do not identify the Java source.

## Source map

| Path | Purpose |
| --- | --- |
| `plugins/org.eclipse.elk.graph` | EMF graph model and graph properties. |
| `plugins/org.eclipse.elk.core` | Recursive layout engine, common options, metadata services, utilities. |
| `plugins/org.eclipse.elk.alg.common` | Shared layout helpers, footprints, and geometric routing. |
| `plugins/org.eclipse.elk.alg.*` | Algorithm providers and their pipelines. |
| `plugins/org.eclipse.elk.graph.json` | JSON import/export, including Xtend source. |
| `plugins/org.eclipse.elk.core.meta` | Metadata language/compiler; algorithm options originate in `*.melk`. |
| `test/org.eclipse.elk.*.test` | JVM test plugins. |
| `test/geometry` | Canonical geometric JSON fixtures shared with elkjs. |
| `build/pom.xml` | Maven/Tycho reactor entry point. |
| `plugins/pom.xml`, `features/`, `test/pom.xml` | Module, distribution, and test registration. |

See `docs/geometric-layout.md` for the geometric provider and `docs/content/documentation/` for general contributor and algorithm documentation.

## Implementation rules

- Follow the existing Java/Xtend style and license headers. Prefer a small change at the owning algorithm stage.
- Preserve public option defaults unless changing them is part of the task. New placement strategies should be opt-in; document corrections that change legacy geometry.
- Preserve graph identity, edge direction, multiplicity, and declared hierarchy. A structural projection or temporary cluster must not silently remove or duplicate caller data.
- Keep top-left coordinates, node centers, and ancestor-relative frames explicit. Translate nodes, routes, bends, and labels together when normalizing bounds.
- Include margins, ports, and labels in footprint/clearance decisions. Validate finite geometry and infeasible fixed dimensions.
- Prefer iterative graph traversal for potentially deep topology. Use deterministic ordering; input permutation invariance applies to `STABLE_ID`, while `MODEL_ORDER` preserves caller order.
- Edit `*.melk` or `*.xtend` sources for generated metadata/code. Regenerate their output through the build. Do not blanket-edit `src-gen`: some generated model sources are tracked and require their own generation workflow.
- Java compiled into elkjs must work with GWT's supported APIs. A JVM compile alone does not establish GWT compatibility; avoid adding reflection, threading, filesystem access, or unsupported JRE APIs to those paths.
- For a new algorithm, check Maven modules, bundle manifest/dependencies/exports, service metadata, the algorithms feature, JVM tests, and the sibling JS source list and manual provider registration.

## Build and tests

Use JDK 17 and Maven. The full suite also needs an `elk-models` checkout. Prefer executable CI configuration in `.github/workflows/ci.yml` over old prose or comments.

From the repository root, set `ELK_MODELS_REPO` to the absolute path of that checkout, then run:

```sh
mvn -f build/pom.xml --fail-at-end --no-transfer-progress \
  -Dtests.paths.elk-repo="$PWD" \
  -Dtests.paths.models-repo="$ELK_MODELS_REPO" \
  -Delk.metadata.documentation.outputPath="$PWD/docs" \
  clean verify
```

- For a focused iteration, use the same reactor and path properties with `-Dtest=TestClassA,TestClassB -DfailIfNoTests=false verify`. Tycho bundle dependencies are not reliably satisfied by a naive `-pl/-am` subset.
- Test code receives the path properties as `ELK_REPO` and `MODELS_REPO`. Missing fixtures/models are not a passing test run.
- If incremental output behaves inconsistently after an API/signature change, use a clean build before debugging stale bytecode.
- Add a regression that distinguishes the faulty behavior when practical. For layout changes, test geometry and graph preservation rather than incidental exact coordinates.
- For Java paths used by elkjs, rebuild and test `../elkjs` with JDK 17 and Node 24. Geometric changes additionally need `scripts/check-geometric-parity.cjs` in elkjs after the JVM fixture export.
- Run the full CI-equivalent verification for broad algorithm/integration changes. Documentation-only changes need path/command and diff checks, not a full reactor build.

## Review and delivery

- Keep cross-repository commits coherent and report the paired revisions, relevant tests, and remaining limitations.
- Do not hand-patch generated worker bundles to make a test pass.
- Do not mix dependency upgrades, version changes, or release pin changes into unrelated layout fixes.
- Keep build caches, temporary toolchains, benchmark output, and local index data out of commits. Inspect ignored/tracked status before staging generated artifacts.

<!-- ZVEC_GREP_START -->
## Workspace retrieval

- Choose the evidence source first. Use workspace retrieval for local code, documents, configuration, and questions grounded in these repositories; use external sources for unrelated or current external facts.
- For an exact identifier, filename, literal, or regex where locating occurrences is sufficient, use `zvec_grep_rg` if available, otherwise `rg`.
- For architecture, relationships, lifecycle, rationale, fuzzy discovery, or cross-file synthesis, use `zvec_grep_search` first. Include known anchors in the semantic query, then use exact searches for focused follow-up.
- Pass the daemon-visible absolute repository `root` on every zvec call. Search the Java and JS repositories separately when the question crosses their boundary.
- Treat sufficient returned snippets as already-read evidence. Open a file only for a detail outside the snippet; avoid broad reads or repeated searches after the evidence is sufficient.
- Read `freshness` and `background_refresh` from results without a status preflight. Use sufficient `served_from_current_index` results while acknowledging possible drift; verify edited source directly.
- For an existence question without an exact anchor, make one focused semantic probe. If it is irrelevant, report that the index did not establish the answer rather than broadening indefinitely.
- If an index is missing and exact search can answer the question, use `rg`. Creating, rebuilding, or dropping persistent indexes requires user authorization; reuse authorization already given in the session.
- Do not delegate merely to locate files. Complete independent local work while a necessary clarification is pending.

<!-- ZVEC_GREP_END -->

## GitNexus operational notes

The block below supplies the required impact/commit gates. Its generated statistics and example comparison base are snapshots; verify current source/index state and use the task's actual base revision. Use the repository name `elk` explicitly when selecting among indexed repositories.

- Prefer MCP tools; use the existing `.gitnexus/run.cjs` or an installed CLI if MCP is unavailable. Verify the runner before relying on its graph.
- Refresh only with user authorization, including authorization already given in the session. Prefer `analyze --index-only` to preserve repository instructions; enable `--pdg` when dependence analysis is needed.
- A dirty worktree can make some CLI versions report `stale` even just after indexing. Check indexed revision, runner identity, incomplete reasons, and source hashes; do not hide user files or treat a status label alone as proof of freshness.
- Graph analysis does not fully resolve Xtend, inline scripts, generated code, or dynamic/cross-language dispatch. For `UNKNOWN`, verify the real source/registration path and relevant tests; never equate zero callers with no impact. Report unavailable or incomplete analysis explicitly.

<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **elk**. Obtain current statistics and freshness from the index instead of treating this file as an index snapshot.

> When an index refresh is authorized, run `node .gitnexus/run.cjs analyze --index-only` from the project root. If the runner is absent, use an available installed CLI or the host's GitNexus tools; do not assume a machine-specific global path.

## Always Do

- **MUST run impact analysis before editing.** Use `impact({target: "symbolName", direction: "upstream"})` (MCP) or `node .gitnexus/run.cjs impact "symbolName" --direction upstream --repo .` (CLI fallback); report callers, processes, and risk. Never substitute grep for graph analysis.
- **MUST analyze graph changes before committing.** Use `detect_changes({scope: "all"})` (MCP) or `node .gitnexus/run.cjs detect-changes --scope all --repo .` (CLI fallback). `partial: true` or `truncated: true` is not a clean check — a zero means unseen, not unaffected; re-run it. For regression review: `detect_changes({scope: "compare", base_ref: "master"})` or `node .gitnexus/run.cjs detect-changes --scope compare --base-ref "master" --repo .`.
- **MUST warn the user** if impact analysis returns HIGH or CRITICAL risk before proceeding with edits.
- **MUST treat `risk: UNKNOWN` as unresolved, not as low.** An empty caller set is not evidence the symbol is unused — it can also mean the callers are not resolvable by the index (plain-object property access, dynamic dispatch, cross-language calls). `impact` pairs `UNKNOWN` with a `riskNote` saying so. Confirm with a text search before treating the symbol as safe to change or delete; do not proceed on the strength of a zero.
- When exploring unfamiliar code, use `query({search_query: "concept"})` to find execution flows instead of grepping. It returns process-grouped results ranked by relevance.
- When you need full context on a specific symbol — callers, callees, which execution flows it participates in — use `context({name: "symbolName"})`.
- For security review, `explain({target: "fileOrSymbol"})` lists taint findings (source→sink flows; needs `analyze --pdg`).

## Never Do

- NEVER edit a function, class, or method before MCP/CLI impact analysis.
- NEVER ignore HIGH or CRITICAL risk warnings from impact analysis, and never read `UNKNOWN` as an all-clear — it means the walk could not answer, which is the one verdict that requires confirming by other means.
- NEVER rename symbols with find-and-replace — use `rename` which understands the call graph.
- NEVER commit before MCP/CLI graph change analysis.

## Resources

| Resource | Use for |
| --- | --- |
| `gitnexus://repo/elk/context` | Codebase overview, check index freshness |
| `gitnexus://repo/elk/clusters` | All functional areas |
| `gitnexus://repo/elk/processes` | All execution flows |
| `gitnexus://repo/elk/process/{name}` | Step-by-step execution trace |

## CLI

| Task | Read this skill file |
| --- | --- |
| Understand architecture / "How does X work?" | `.claude/skills/gitnexus-exploring/SKILL.md` |
| Blast radius / "What breaks if I change X?" | `.claude/skills/gitnexus-impact-analysis/SKILL.md` |
| Trace bugs / "Why is X failing?" | `.claude/skills/gitnexus-debugging/SKILL.md` |
| Rename / extract / split / refactor | `.claude/skills/gitnexus-refactoring/SKILL.md` |
| Tools, resources, schema reference | `.claude/skills/gitnexus-guide/SKILL.md` |
| Index, status, clean, wiki CLI commands | `.claude/skills/gitnexus-cli/SKILL.md` |

<!-- gitnexus:end -->
