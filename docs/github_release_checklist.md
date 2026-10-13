# GitHub Release & Publication Checklist

This checklist provides step-by-step guidance for safely publishing the **AI-Based Autonomous Navigation System** to a public or private GitHub repository.

> **Important**: Do not initialize git, add remotes, or push code until you have reviewed this checklist and are ready to publish.

---

## 1. Pre-Publication Security & Repository Hygiene Review

Before running any git commands, ensure that sensitive information and unnecessary build artifacts are excluded:

- [ ] **Check for sensitive data**:
  - No API keys, passwords, or personal access tokens exist in the repository.
  - No personal absolute filesystem paths or private credentials are hardcoded.
- [ ] **Verify `.gitignore`**:
  - Python caches (`__pycache__/`, `*.pyc`, `.pytest_cache/`) are ignored.
  - Virtual environments (`venv/`, `.venv/`, `env/`) are ignored.
  - Scratch/temporary directories (`scratch/`) are ignored.
  - Binary bundles (`*.zip`, `*.tar.gz`) are ignored.
  - Core benchmark data (`outputs/metrics/benchmark_results*.json`) is preserved while ad-hoc outputs are ignored.
- [ ] **Verify Dependencies**:
  - `requirements.txt` matches `pyproject.toml`.
  - Core dependencies (`numpy`, `pygame`, `matplotlib`, `pyyaml`, `pytest`) are explicitly documented.

---

## 2. GitHub Repository Creation (Web Interface)

1. Navigate to [GitHub New Repository](https://github.com/new).
2. Choose repository settings:
   - **Repository name**: `AI-Based-Autonomous-Navigation-System` (or your preferred name).
   - **Description**: `A simulation-first autonomous mobile robot navigation system in Python featuring A*, Dijkstra, clearance-aware costmaps, and collision-safe path smoothing.`
   - **Visibility**: Public (recommended for portfolio/placement) or Private.
3. **Important**:
   - **Do NOT** check "Add a README file" (the project already has a comprehensive `README.md`).
   - **Do NOT** add a `.gitignore` template (the project has a tailored `.gitignore`).
   - **Do NOT** choose a license template (the project includes an existing MIT [LICENSE](../LICENSE)).
4. Click **Create repository**.
5. Copy the HTTPS or SSH repository URL provided on the setup page (e.g., `https://github.com/<YOUR-USERNAME>/AI-Based-Autonomous-Navigation-System.git`).

---

## 3. Local Git Initialization & Push Commands

Choose the section below that matches your local environment's current state.

### Scenario A: Initializing a New Git Repository from Scratch

Use this if `git status` indicates that no Git repository has been created yet:

```bash
# 1. Initialize local repository
git init

# 2. Stage all hygiene-approved files
git add .

# 3. Check staged files to ensure no scratch or zip files were included
git status

# 4. Create initial commit
git commit -m "feat: initial commit of AI-Based Autonomous Navigation System"

# 5. Set default branch to main
git branch -M main

# 6. Add GitHub remote (replace with your actual GitHub URL)
git remote add origin https://github.com/<YOUR-USERNAME>/AI-Based-Autonomous-Navigation-System.git

# 7. Push to GitHub
git push -u origin main
```

### Scenario B: Existing Repository Re-linking

Use this if a Git repository was previously initialized locally and you need to link or update the remote URL:

```bash
# Verify current remote status
git remote -v

# If origin already exists and points to an old location, update it:
git remote set-url origin https://github.com/<YOUR-USERNAME>/AI-Based-Autonomous-Navigation-System.git

# Or if no remote exists yet:
git remote add origin https://github.com/<YOUR-USERNAME>/AI-Based-Autonomous-Navigation-System.git

# Stage, commit, and push
git add .
git commit -m "docs: finalize repository for public release"
git push -u origin main
```

---

## 4. Documentation & Placeholder Checks

- [ ] **Replace GitHub URL Placeholder**:
  After creating the remote, update the placeholder in documentation files:
  - In [`README.md`](../README.md): Replace `YOUR-USERNAME` in the clone command:
    ```bash
    git clone https://github.com/<YOUR-USERNAME>/AI-Based-Autonomous-Navigation-System.git
    ```
  - In [`docs/setup.md`](setup.md): Replace `your-username` with your real username.
- [ ] **Review README Links**:
  - Verify that relative links to documentation files (`docs/architecture.md`, `docs/algorithms.md`, `docs/testing.md`, etc.) resolve properly.
- [ ] **Capture and Embed Real Screenshots (Optional but Recommended)**:
  - Follow [`docs/demo_checklist.md`](demo_checklist.md) to capture genuine simulation screenshots from the Pygame window.
  - Save captured images in `assets/images/`.
  - Add image markdown links to `README.md` under a `## Demo` section.
  - Avoid using synthetic mockups or AI-generated application windows.

---

## 5. Post-Publication Verification Steps

Once pushed to GitHub, perform these checks from a clean environment or browser:

1. **GitHub Repository Landing Page**:
   - Confirm `README.md` renders properly with code syntax highlighting, markdown tables, and the Mermaid architecture diagram.
   - Confirm the MIT license badge/label displays on the repository sidebar.
   - Confirm that temporary directories (`scratch/`), binary archives (`*.zip`), and `__pycache__` folders are **not** present in the file tree.
2. **Clean Clone Verification (Smoke Test)**:
   - In a separate temporary folder, run:
     ```bash
     git clone https://github.com/<YOUR-USERNAME>/AI-Based-Autonomous-Navigation-System.git test-clone
     cd test-clone
     python -m venv venv
     # Windows: .\venv\Scripts\Activate.ps1 | Linux/macOS: source venv/bin/activate
     pip install -r requirements.txt
     pip install -e .
     pytest -q
     python scripts/validate_consistency.py
     python main.py --headless
     ```
   - Confirm all 161 tests pass and simulation executes cleanly.

---

## 6. Audit & External Review Note

- Publishing the repository to GitHub allows independent reviewers (including external AI auditors or human reviewers) to clone the repository directly and execute the tests on their own systems.
- The technical archive audit remains **OPEN** until an external reviewer reproduces the test suite and simulation runs on the matching deliverable.
