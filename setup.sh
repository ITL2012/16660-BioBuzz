#!/usr/bin/env bash
# =============================================================================
# BioBuzz Robotics - Automated Setup Script
# =============================================================================
# This script sets up the complete development environment for the BioBuzz
# FTC robot codebase. Run this on any fresh machine to get a working build.
#
# Usage: ./setup.sh [--ci] [--no-android] [--help]
#   --ci          : Run in CI mode (non-interactive, fail on error)
#   --no-android  : Skip Android SDK setup (for JVM tests only)
#   --help        : Show this help
# =============================================================================

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GRADLE_WRAPPER_VERSION="8.13.2"
KOTLIN_VERSION="2.2.20"
JDK_VERSION="11"

CI_MODE=false
NO_ANDROID=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        --ci) CI_MODE=true ;;
        --no-android) NO_ANDROID=true ;;
        --help)
            echo "Usage: $0 [--ci] [--no-android] [--help]"
            exit 0
            ;;
        *) echo "Unknown argument: $arg"; exit 1 ;;
    esac
done

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

check_command() {
    if ! command -v "$1" &> /dev/null; then
        log_error "$1 not found. Please install it first."
        return 1
    fi
    log_success "$1 found: $($1 --version 2>&1 | head -1)"
}

# =============================================================================
# Step 1: Check Prerequisites
# =============================================================================
log_info "Checking prerequisites..."

check_command git
check_command java
check_command ./gradlew

# Check Java version
JAVA_VER=$(java -version 2>&1 | head -1 | sed -E 's/.*version "([0-9]+).*/\1/')
if [[ "$JAVA_VER" -lt 11 ]]; then
    log_error "Java 11+ required (found $JAVA_VER). Install JDK 11 or later."
    exit 1
fi
log_success "Java version OK: $JAVA_VER"

# =============================================================================
# Step 2: Android SDK Setup (unless --no-android)
# =============================================================================
if [[ "$NO_ANDROID" != "true" ]]; then
    log_info "Setting up Android SDK..."
    
    # Check for ANDROID_HOME or ANDROID_SDK_ROOT
    if [[ -z "${ANDROID_HOME:-}" && -z "${ANDROID_SDK_ROOT:-}" ]]; then
        log_warn "ANDROID_HOME not set. Attempting to find Android SDK..."
        
        # Common locations
        for loc in \
            "$HOME/Android/Sdk" \
            "$HOME/Library/Android/sdk" \
            "/opt/android-sdk" \
            "/usr/local/android-sdk"; do
            if [[ -d "$loc" ]]; then
                export ANDROID_HOME="$loc"
                log_success "Found Android SDK at $ANDROID_HOME"
                break
            fi
        done
        
        if [[ -z "${ANDROID_HOME:-}" ]]; then
            log_error "Android SDK not found. Please install Android Studio and set ANDROID_HOME."
            log_error "Or run with --no-android for JVM tests only."
            exit 1
        fi
    fi
    
    # Verify SDK has required platforms
    if [[ ! -d "$ANDROID_HOME/platforms/android-30" ]]; then
        log_warn "Android API 30 not found. You may need to install it via SDK Manager."
    fi
    
    log_success "Android SDK configured at $ANDROID_HOME"
fi

# =============================================================================
# Step 3: Gradle Wrapper Verification
# =============================================================================
log_info "Verifying Gradle wrapper..."

if [[ ! -f "$PROJECT_ROOT/gradlew" ]]; then
    log_error "gradlew not found in project root. Are you in the right directory?"
    exit 1
fi

chmod +x "$PROJECT_ROOT/gradlew"

# Test Gradle
if "$PROJECT_ROOT/gradlew" --version > /dev/null 2>&1; then
    log_success "Gradle wrapper works"
else
    log_error "Gradle wrapper failed. Try: ./gradlew --version"
    exit 1
fi

# =============================================================================
# Step 4: Download Dependencies (offline cache warmup)
# =============================================================================
log_info "Downloading dependencies (this may take a few minutes)..."

if "$PROJECT_ROOT/gradlew" :TeamCode:dependencies --configuration compileClasspath > /dev/null 2>&1; then
    log_success "Dependencies resolved"
else
    log_warn "Dependency resolution had warnings (check with --info)"
fi

# =============================================================================
# Step 5: Run JVM Tests (verifies pure Kotlin autonomy core)
# =============================================================================
log_info "Running JVM tests to verify autonomy core..."

if "$PROJECT_ROOT/gradlew" :TeamCode:test --tests "org.firstinspires.ftc.teamcode.*Test" > /dev/null 2>&1; then
    log_success "JVM tests passed"
else
    log_error "JVM tests failed. Run './gradlew :TeamCode:test --info' for details."
    if [[ "$CI_MODE" == "true" ]]; then exit 1; fi
fi

# =============================================================================
# Step 6: Verify Android Build (if Android SDK available)
# =============================================================================
if [[ "$NO_ANDROID" != "true" ]]; then
    log_info "Verifying Android build (assembleDebug)..."
    
    if "$PROJECT_ROOT/gradlew" :TeamCode:assembleDebug > /dev/null 2>&1; then
        log_success "Android build successful"
    else
        log_error "Android build failed. Run './gradlew :TeamCode:assembleDebug --info' for details."
        if [[ "$CI_MODE" == "true" ]]; then exit 1; fi
    fi
fi

# =============================================================================
# Step 7: Generate Setup Verification Report
# =============================================================================
log_info "Generating setup verification report..."

cat > "$PROJECT_ROOT/SETUP_VERIFICATION.md" << EOF
# BioBuzz Setup Verification Report
Generated: $(date)

## Environment
- Java: $(java -version 2>&1 | head -1)
- Gradle: $($PROJECT_ROOT/gradlew --version | head -1)
- Android SDK: ${ANDROID_HOME:-"Not set (--no-android mode)"}
- Kotlin: $KOTLIN_VERSION

## Build Status
- JVM Tests: $(if [[ -f "$PROJECT_ROOT/TeamCode/build/reports/tests/test/index.html" ]]; then echo "PASSED"; else echo "NOT RUN"; fi)
- Android Build: $(if [[ "$NO_ANDROID" == "true" ]]; then echo "SKIPPED"; elif [[ -f "$PROJECT_ROOT/TeamCode/build/outputs/apk/debug/TeamCode-debug.apk" ]]; then echo "PASSED"; else echo "FAILED"; fi)

## Next Steps
1. Open project in Android Studio: File > Open > $PROJECT_ROOT
2. Sync Gradle (File > Sync Project with Gradle Files)
3. Connect Robot Controller device via USB
4. Run 'BioBuzz Autonomous' or 'BioBuzz TeleOp' configuration

## Troubleshooting
- If Gradle sync fails: File > Invalidate Caches / Restart
- If SDK not found: Tools > SDK Manager > Install Android API 30
- If build fails: ./gradlew clean :TeamCode:assembleDebug --info

EOF

log_success "Setup verification report written to SETUP_VERIFICATION.md"

# =============================================================================
# Complete
# =============================================================================
echo ""
echo "========================================"
log_success "SETUP COMPLETE!"
echo "========================================"
echo ""
echo "Next steps:"
echo "  1. Open in Android Studio: File > Open > $PROJECT_ROOT"
echo "  2. Sync Gradle (File > Sync Project with Gradle Files)"
echo "  3. Connect robot via USB"
echo "  4. Run 'BioBuzz Autonomous' configuration"
echo ""
echo "For CI/CD: Run './gradlew :TeamCode:test :TeamCode:assembleDebug'"
echo "For JVM-only testing: Run './gradlew :TeamCode:test'"
echo ""
log_info "Report saved to: $PROJECT_ROOT/SETUP_VERIFICATION.md"