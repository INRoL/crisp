const select = (selector, root = document) => root.querySelector(selector);
const selectAll = (selector, root = document) => [
  ...root.querySelectorAll(selector),
];

const commandOverrides = {
  windows: `git clone https://github.com/INRoL/crisp.git
cd crisp
cmake -S . -B build -Dcrisp_use_bundled_eigen=ON
cmake --build build --config Release --target 01_franka
.\\build\\examples\\Release\\01_franka.exe`,
};

function setRelatedPublications(open) {
  const button = select(".more-works-btn");
  const dropdown = select(".more-works-dropdown");

  if (!button || !dropdown) return;

  button.classList.toggle("active", open);
  button.setAttribute("aria-expanded", String(open));
  dropdown.classList.toggle("show", open);
}

function copyText(button, targetId) {
  const code = document.getElementById(targetId);
  const label = button && select(".copy-text", button);

  if (!code || !button || !label) return;

  const originalLabel = label.textContent;

  navigator.clipboard.writeText(code.innerText.trim()).then(() => {
    button.classList.add("copied");
    label.textContent = "Copied";

    window.setTimeout(() => {
      button.classList.remove("copied");
      label.textContent = originalLabel;
    }, 2000);
  });
}

function detectCommandPlatform() {
  const platform = (
    navigator.userAgentData?.platform
    || navigator.platform
    || ""
  ).toLowerCase();

  return platform.includes("win") ? "windows" : "unix";
}

function setCommandPlatform(platform) {
  const code = select("#getting-started-code");

  if (!code) return;

  if (!code.dataset.defaultCommands) {
    code.dataset.defaultCommands = code.textContent.trim();
  }

  code.textContent = platform === "windows"
    ? commandOverrides.windows
    : code.dataset.defaultCommands;

  selectAll(".command-platform-btn").forEach((button) => {
    const active = button.dataset.commandPlatform === platform;

    button.classList.toggle("active", active);
    button.setAttribute("aria-pressed", String(active));
  });
}

function initPage() {
  const moreWorksButton = select(".more-works-btn");
  const dropdown = select(".more-works-dropdown");

  moreWorksButton?.addEventListener("click", () => {
    setRelatedPublications(!moreWorksButton.classList.contains("active"));
  });

  select(".close-btn")?.addEventListener("click", () => {
    setRelatedPublications(false);
  });

  select(".copy-bibtex-btn")?.addEventListener("click", (event) => {
    copyText(event.currentTarget, "bibtex-code");
  });

  selectAll(".copy-code-btn").forEach((button) => {
    button.addEventListener("click", () => {
      copyText(button, button.dataset.copyTarget);
    });
  });

  selectAll(".command-platform-btn").forEach((button) => {
    button.addEventListener("click", () => {
      setCommandPlatform(button.dataset.commandPlatform);
    });
  });

  setCommandPlatform(detectCommandPlatform());

  document.addEventListener("click", (event) => {
    const shouldClose = (
      dropdown?.classList.contains("show")
      && !dropdown.contains(event.target)
      && !moreWorksButton?.contains(event.target)
    );

    if (shouldClose) {
      setRelatedPublications(false);
    }
  });

  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") setRelatedPublications(false);
  });
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initPage);
} else {
  initPage();
}
