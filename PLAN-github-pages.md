# Deploying WhatIfOrbit to GitHub Pages

The app is entirely client-side (Rust → WASM, no backend), so GitHub Pages can host it as
static files. This plan covers the build, the CI workflow, the subpath gotcha, and
verification.

## Prerequisites (repo state)

- The repo currently has **only the `Initial` commit** and **no remote**. Before any of
  this works: create a GitHub repo, land the current `target-orbit` work onto `main`
  (commit it — nothing is committed yet), and `git remote add origin …` + push.
- Decide the deploy trigger: **push to `main`** (recommended) so every merge publishes.

## The one real gotcha: the subpath

A project Pages site serves at `https://<user>.github.io/<repo>/` — assets live under
`/<repo>/`, **not** root. Two independent asset paths must both respect that subpath:

1. **Trunk-injected links** (the `.js` loader and `_bg.wasm`): fixed by building with
   `--public-url`. Pass it as a **CLI flag in CI only** (not in Trunk.toml) so local
   `trunk serve` keeps working at `http://127.0.0.1:8080/`.
   - Primary: `--public-url /whatiforbit/` (matches the repo name).
   - Repo-name-independent alternative if you prefer: `--public-url ./` (relative). Try
     the explicit one first; it's the documented-reliable pattern.
2. **Bevy's runtime asset fetches** (`earth.jpg` via `AssetServer::load("earth.jpg")`):
   Bevy fetches `assets/earth.jpg` **relative to the page**, so under
   `…/whatiforbit/index.html` it resolves to `…/whatiforbit/assets/earth.jpg`.
   **Verified locally** (2026-07): built with `--public-url /whatiforbit/`, staged the
   dist tree under a `/whatiforbit/` path, served it with a plain static server
   (`python3 -m http.server`, mimicking Pages), and confirmed the Earth texture renders
   and the network log shows `GET /whatiforbit/assets/earth.jpg → 200`. So the relative
   fetch resolves correctly under the subpath — no `<base>` tag needed.
   `AssetMetaCheck::Never` (already set) means no `.meta` probes either. If a future asset
   is added via an absolute path and 404s, the fix is a `<base href="/whatiforbit/">`.

Everything else already works on Pages: `.wasm` is served as `application/wasm`, the
`?demo` query param reads fine, CelesTrak CORS is open (verified), and our WASM build is
**single-threaded** — so we don't need the COOP/COEP headers Pages can't set.

## GitHub Actions workflow

Add `.github/workflows/deploy-pages.yml`. Shape:

```yaml
name: Deploy to GitHub Pages
on:
  push:
    branches: [main]
  workflow_dispatch:
permissions:
  contents: read
  pages: write
  id-token: write
concurrency:
  group: pages
  cancel-in-progress: true
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable
        with:
          targets: wasm32-unknown-unknown
      - uses: Swatinem/rust-cache@v2       # caches cargo registry + target
      - uses: jetli/trunk-action@v0.5.0    # prebuilt trunk (fast); pins version
      - run: trunk build --release --public-url /whatiforbit/
      - run: touch dist/.nojekyll          # disable Jekyll processing
      - uses: actions/upload-pages-artifact@v3
        with:
          path: dist
  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - id: deployment
        uses: actions/deploy-pages@v4
```

Notes:
- **Trunk auto-downloads `wasm-bindgen` and `wasm-opt`** at build time (observed locally),
  so no explicit install step — but the first CI run pays that download cost.
- **`Swatinem/rust-cache`** is the big time-saver; Bevy is heavy to compile. Expect a cold
  first run of several minutes, cached runs much faster.
- **`.nojekyll`** via `touch` after build (simplest); alternatively a
  `rel="copy-file"` link in index.html.

## Repo settings (one-time)

1. Push the workflow to `main`.
2. Settings → Pages → **Source: GitHub Actions** (not "Deploy from a branch").
3. First run publishes to `https://<user>.github.io/whatiforbit/`. HTTPS is automatic.

## Verification checklist (first deploy)

- Page loads, Bevy canvas renders (not blank/black).
- **Earth texture appears** — the subpath asset test. If blank, apply the `<base>` fix.
- `…/whatiforbit/?demo&target` runs the canned solve — confirms the query-param path.
- "Fetch from CelesTrak" by NORAD ID succeeds (CORS confirmed open, but verify live).
- No 404s in the Network tab for `_bg.wasm`, the `.js`, or `assets/earth.jpg`.

## Optional, non-blocking

- **Bundle size**: the WASM is ~33 MB uncompressed (Pages gzips it to ~1/3 over the wire,
  but it's still the dominant load). If load time matters: switch `data-wasm-opt="s"` →
  `"z"` in index.html, and trim default Bevy features. Tracked in PLAN.md's parking lot.
- **Custom domain**: add a `CNAME` file (via `rel="copy-file"`) and configure DNS; this
  also moves you to root `/`, sidestepping the subpath entirely.
- **Cache busting**: Trunk already hashes output filenames, so deploys invalidate cleanly.

## Risks

| Risk | Mitigation |
|---|---|
| Bevy asset fetch breaks under the subpath (blank Earth) | **Retired** — verified locally against a Pages-like static server; Earth renders under `/whatiforbit/` |
| Cold CI builds are slow (Bevy) | `rust-cache`; prebuilt trunk; accept a slow first run |
| Large WASM download on first visit | gzip helps; optional `-Oz` + feature trim; not blocking for an MVP demo |
| CelesTrak changes CORS policy | Paste-TLE always works offline; would only lose fetch-by-ID |
