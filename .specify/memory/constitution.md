<!-- Sync Impact Report:
Version change: 1.2.0 → 1.3.0
Modified principles: "I. Technical Accuracy", "II. Clarity for Learning", "III. Practical and Hands-On Orientation", "IV. Consistency Across Chapters" (content refined)
Added sections: IX. Workflow Compliance standards
Removed sections: None
Templates requiring updates: ✅ .specify/templates/plan-template.md, ✅ .specify/templates/spec-template.md, ✅ .specify/templates/tasks-template.md, ⚠ .specify/templates/commands/*.md
Follow-up TODOs: None
-->

# AI/Spec-Driven Book Creation Constitution

## Core Principles

### I. Technical Accuracy
All explanations, definitions, examples, and workflows must be technically correct and aligned with authoritative developer documentation. AI-generated content must be validated against official sources (Docusaurus docs, GitHub Pages docs, Spec-Kit Plus repo, Node.js documentation, etc.).

### II. Clarity for Learning
Writing must target beginners to intermediate developers. Concepts must be explained progressively, with simple language and layered depth. Every chapter should reinforce understanding through examples, diagrams, and checklists.

### III. Practical and Hands-On Orientation
The book should focus on building, deploying, and maintaining a real-world documentation site using Docusaurus. Each section must contain step-by-step guides, code snippets, and commands that readers can follow directly.

### IV. Consistency Across Chapters
Terminology, command formatting, and explanation style must remain uniform throughout the book. All code blocks must be tested before inclusion.

## Key Standards

### V. Documentation Quality
All steps must be validated in a real Docusaurus + GitHub Pages environment. Screenshots, examples, and folder structures must reflect actual project states.

### VI. Source Verification
Whenever referencing tools or frameworks, cite:
- Official documentation
- GitHub repositories
- Verified technical blogs or RFCs
No unverified claims or ambiguous instructions.

### VII. Style Guidelines
Writing style: clear, concise, and first-person plural ("we"). Code formatting must follow proper syntax highlighting. Explanations must connect concepts to real usage scenarios.

### VIII. Book Structure Standards
Each chapter must include:
- Objective
- Explanation
- Code examples
- Common errors and troubleshooting
- Summary
- References (if needed)

## Constraints
Book Format:
- Written in Markdown for Docusaurus
- Must be compatible with Docusaurus v2
- All assets stored inside /docs and /static folders

Tone: instructional, accessible, developer-friendly

Content Type: must include tutorials, conceptual chapters, code walkthroughs, and deployment guides

Tooling: Content must be generated using Spec-Kit Plus + Claude Code workflow

Deployment: Final book must be deployable to GitHub Pages without errors

## Success Criteria

### Functional Output
- The book builds successfully using npm run build
- Deployment to GitHub Pages works flawlessly
- No broken links in the generated site

### Content Quality
- Clear, actionable, and free of contradictions
- Commands tested and verified
- No outdated instructions

### Learning Impact
- A reader with basic programming knowledge can fully create and deploy their own Docusaurus site
- Chapters build progressively from fundamentals to advanced topics

### Workflow Compliance
- All chapters generated with Spec-Kit Plus specs
- Constitution principles followed throughout
- Claude Code used for code generation and refactoring as needed

## Governance
This constitution governs all development and content creation for the AI/Spec-Driven Book Creation project. All PRs/reviews must verify compliance with these principles. Changes to this constitution require documented justification and approval from project maintainers.

**Version**: 1.3.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
