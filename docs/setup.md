# Environment Setup & Installation

This project is tested with Python 3.11+.

## Prerequisites

- Python 3.11, 3.12, or 3.13 installed
- `pip` package manager
- Optional: Git for version control

## Installation Steps

### 1. Clone the repository
```bash
git clone https://github.com/your-username/AI-Based-Autonomous-Navigation-System.git
cd AI-Based-Autonomous-Navigation-System
```

### 2. Create a virtual environment (recommended)
```bash
python -m venv venv
```

Activate the environment:
- **Windows (Command Prompt / PowerShell)**:
  ```powershell
  .\venv\Scripts\Activate.ps1
  ```
- **Linux / macOS**:
  ```bash
  source venv/bin/activate
  ```

### 3. Install dependencies

Install the core dependencies:
```bash
pip install -r requirements.txt
```

Alternatively, install in editable development mode:
```bash
pip install -e .
```

### 4. Verify installation

Run the automated test suite to ensure all modules are operating properly:
```bash
pytest
```
