<!--
Sync Impact Report:
Version change: none -> 1.0.0
List of modified principles:
  - All principles are new based on user input.
Added sections:
  - Project
  - Core Principles
  - Key Standards
  - Constraints
  - Success Criteria
  - Deliverables
  - Quick-Start Action Plan
Removed sections:
  - None (as this is the initial formal constitution).
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
  - .gemini/commands/sp.adr.toml ✅ updated
  - .gemini/commands/sp.analyze.toml ✅ updated
  - .gemini/commands/sp.checklist.toml ✅ updated
  - .gemini/commands/sp.clarify.toml ✅ updated
  - .gemini/commands/sp.constitution.toml ✅ updated
  - .gemini/commands/sp.git.commit_pr.toml ✅ updated
  - .gemini/commands/sp.implement.toml ✅ updated
  - .gemini/commands/sp.phr.toml ✅ updated
  - .gemini/commands/sp.plan.toml ✅ updated
  - .gemini/commands/sp.specify.toml ✅ updated
  - .gemini/commands/sp.tasks.toml ✅ updated
Follow-up TODOs: None
-->
# /sp.constitution

## Project
**Title:** *Physical AI & Humanoid Robotics* – a technical book that introduces the theory, hardware, and software foundations of embodied artificial intelligence.  
**Toolchain:**  
- **Spec‑Kit‑Plus** (https://github.com/panaversity/spec-kit-plus) – for structured authoring & validation.  
- **Gemini‑CLI** – to render Spec‑Kit‑Plus sources into Docusaurus markdown.  
- **Docusaurus** – static‑site generator for the book website.  
- **GitHub Pages** – hosting the final site (`username.github.io/repo-name`).
- Docusaurus for book website generation.
- The project must be authored using Docusaurus docs structure.

---

## Core Principles
| # | Principle | Why it matters for Physical AI & Humanoid Robotics |
|---|-----------|---------------------------------------------------|
| 1 | **Scientific Accuracy** | Every claim about sensors, actuators, control loops, or AI algorithms must be traceable to primary research or vendor datasheets. |
| 2 | **Clarity for Engineers** | The audience is computer‑science & robotics students; explanations must be concise, with diagrams and code snippets that compile out‑of‑the‑box. |
| 3 | **Reproducibility** | All code, simulation setups, and hardware bill‑of‑materials (BOM) are version‑controlled and runnable on a standard Linux VM. |
| 4 | **Rigor & Peer Review** | Preference for peer‑reviewed journal/conference papers; pre‑prints are allowed if clearly marked. |
| 5 | **Open‑Source Ethos** | All assets (text, figures, code) are released under an OSI‑approved license (e.g., MIT/CC‑BY‑4.0). |

---

## Key Standards
- **Citation style:** APA 7th edition.  
- **Source mix:** ≥ 50 % peer‑reviewed articles; remaining can be white‑papers, standards (IEEE, ISO), or reputable technical blogs.  
- **Plagiarism tolerance:** 0 % (run through a plagiarism detector before each release).  
- **Readability:** Flesch‑Kincaid Grade **10‑12** (≈ college‑level).  
- **Code style:** PEP‑8 for Python, Google C++ Style for C/C++; all snippets must be lint‑checked.  
- **Figure format:** SVG for vector diagrams, PNG (300 dpi) for raster images; each figure must have a caption and a source reference.  

---

## Constraints
| Constraint | Specification |
|------------|----------------|
| **Word count** | **5,000 – 7,000** words (excluding front‑matter, bibliography, and appendices). |
| **Minimum sources** | **15** distinct references. |
| **File format** | Final deliverable: **PDF** (generated from Docusaurus via `npm run build && npx docusaurus-pdf`). PDF must embed clickable citations. |
| **Version control** | All source files (`*.md`, `*.yml`, code, figures) live in a **Git** repo; each major chapter is a separate branch for peer review. |
| **Build reproducibility** | `gemini build` must complete without manual edits; CI pipeline (GitHub Actions) runs `gemini lint`, `gemini test`, and `docusaurus build`. |
| **License** | **MIT** for code, **CC‑BY‑4.0** for text & figures. |
| **Docusaurus Init** | A Docusaurus project must be initialized using: `npx create-docusaurus@latest <book-name> classic` |
| **Module Storage** | All modules/chapters must be stored inside `/docs/<module>/` directories. |

---

## Success Criteria
- **Verification:** Every factual statement is linked to a citation; a script (`scripts/verify‑citations.js`) checks that each citation resolves to a DOI/URL and that the source meets the “peer‑reviewed ≥ 50 %” rule.  
- **Plagiarism:** `scripts/run‑plagiarism.sh` (uses `turnitin-cli` or `copydetect`) returns **0 %** similarity.  
- **Readability:** `scripts/readability‑check.sh` (uses `textstat`) reports a Flesch‑Kincaid grade between **10‑12**.  
- **Build:** CI pipeline passes **all** steps (`lint → test → build → pdf → deploy`). The generated PDF is under **7 MB** and all hyperlinks work.  
- **Deployment:** The Docusaurus site is live on **GitHub Pages** (`https://<username>.github.io/<repo>/`) and automatically updates on every merge to `main`.
- `npm run build` for Docusaurus must succeed inside CI.
- GitHub Pages deployment of the Docusaurus site must be functional.  

---

## Deliverables (Checklist)

| Item | Description | Location in repo |
|------|-------------|------------------|
| **Constitution** | This document – the governance backbone. | `sp.constitution.md` |
| **Chapter specs** | One Spec‑Kit‑Plus file per chapter (e.g., `01_introduction.sp`). | `specs/` |
| **Code examples** | Fully runnable notebooks / scripts. | `code/` |
| **BOM & CAD** | Parts list + optional Fusion 360 files. | `hardware/` |
| **Figures** | SVG/PNG with captions. | `static/img/` |
| **CI workflow** | GitHub Actions YAML. | `.github/workflows/ci.yml` |
| **Docusaurus config** | Site metadata, theme, plugins. | `docusaurus.config.js` |
| **PDF build script** | Generates final PDF. | `scripts/build‑pdf.sh` |
| **Docusaurus Project** | The Docusaurus project folder, sidebar configuration, and build scripts. | `(root)` |
| **License files** | `LICENSE`, `CODE_OF_CONDUCT.md`. | Root |

---

## Quick‑Start Action Plan

1. **Create the repo** (public or private) and enable GitHub Pages.  
## Project Governance & Roles
- Every major chapter must be reviewed by at least one peer before merging into main branch.
- All code examples and simulations must pass independent reviewer testing.
- Contributors must sign a Contributor License Agreement (CLA) before submitting substantial content.
- Contribution guidelines and code of conduct must be maintained in `CONTRIBUTING.md`.

## Risk Management & Dependencies
- All external dependencies (hardware, GPUs, simulation tools) must be documented in `hardware/` or `tools/`.
- For every dependency, a fallback or mitigation plan must exist in case of unavailability.
- A Risk Register must be maintained highlighting major project risks (e.g. hardware costs, licensing, build failures).

## Work Breakdown & Milestones
- Milestones: Module 1 draft, Module 2 draft, Module 3 draft, Module 4 draft, RAG integration, First full build & deploy, Beta release.
- A Work Breakdown Structure (WBS) must be maintained to ensure 100% of project scope is covered. :contentReference[oaicite:3]{index=3}

## QA & Review Process
- All code snippets must pass automated tests and linting.
- All diagrams and figures must have alt-text / captions.
- All factual claims must be verified against primary or peer-reviewed sources.
- Peer review must be completed before publication.

## Versioning & Change Management
- Use semantic versioning (e.g. v1.0.0) for major releases.
- Maintain a CHANGELOG.md for any updates, fixes or additions.
- No direct edits on published pages — all changes must go through spec → plan → tasks → implement workflow.

## Accessibility & Internationalization
- Provide captions and alt-text for all images & diagrams.
- When translating content (e.g. Urdu), maintain original structure and accessibility.
- Book theme must support responsive layout and RTL languages if needed.

## Privacy & Data Security (for RAG chatbot / login)
- User data must be stored securely in Neon Postgres; sensitive data must be encrypted.
- No user data should be logged unnecessarily.
- Must comply with GDPR-like consent; provide privacy policy and data usage disclosure.

## Maintenance & Update Policy
- All fixes or updates post-publication must be documented as a new version.
- Errata must be logged in an ERRATA.md and merged through standard spec-driven workflow.