# Implementation Plan: Docusaurus UI Upgrade

**Branch**: `005-docusaurus-ui-upgrade` | **Date**: 2024-12-24 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-docusaurus-ui-upgrade/spec.md`

## Summary

Upgrade the visual design, usability, and navigation of the Physical AI & Robotics Docusaurus book site. Implementation uses Infima CSS variable overrides as the primary theming mechanism, with minimal component swizzling limited to homepage content replacement. The upgrade targets WCAG AA accessibility compliance and Core Web Vitals performance standards.

## Technical Context

**Language/Version**: TypeScript 5.6, React 19, Node.js 20+
**Primary Dependencies**: Docusaurus 3.9.2, Infima CSS Framework, Prism React Renderer
**Storage**: N/A (static site)
**Testing**: Manual visual testing, Lighthouse audits, WebAIM Contrast Checker
**Target Platform**: Modern browsers (Chrome, Firefox, Safari, Edge), responsive 320px-2560px
**Project Type**: Web application (Docusaurus static site)
**Performance Goals**: LCP <2.5s, FID <100ms, CLS <0.1 (Core Web Vitals "Good")
**Constraints**: No content changes, preserve all routes, WCAG AA compliance
**Scale/Scope**: ~10-15 files modified, 4 modules of documentation content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The project constitution uses placeholder values. Applying general best practices:

| Principle | Status | Notes |
|-----------|--------|-------|
| Smallest viable change | PASS | CSS variables + minimal component edits |
| No breaking changes | PASS | All existing routes preserved |
| Testable outcomes | PASS | Lighthouse, contrast tools, visual review |
| Code quality | PASS | TypeScript, standard Docusaurus patterns |

**Gate Status**: PASSED - No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/005-docusaurus-ui-upgrade/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0: Technical research
├── data-model.md        # Phase 1: Theme configuration model
├── quickstart.md        # Phase 1: Implementation guide
├── contracts/
│   └── css-variables.md # CSS variable interface contract
├── checklists/
│   └── requirements.md  # Specification quality checklist
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (repository root)

```text
frontend_book/
├── docusaurus.config.ts     # Site configuration (minor updates)
├── sidebars.ts              # Sidebar config (unchanged)
├── package.json             # Dependencies (unchanged)
├── src/
│   ├── css/
│   │   └── custom.css       # PRIMARY: Theme variable overrides
│   ├── pages/
│   │   ├── index.tsx        # Homepage layout updates
│   │   └── index.module.css # Homepage styles
│   └── components/
│       └── HomepageFeatures/
│           ├── index.tsx    # Feature cards content
│           └── styles.module.css
└── static/
    └── img/                  # May need book-specific assets
```

**Structure Decision**: Web application (frontend-only). All changes are within the existing `frontend_book/` Docusaurus project. No new directories needed.

## Implementation Approach

### Phase 1: Typography & Color Foundation
1. Update `custom.css` with new Infima variable values
2. Define primary color palette (7 shades) for light mode
3. Define dark mode color palette
4. Set typography variables (18px base, 1.6 line height)
5. Test contrast ratios meet WCAG AA

### Phase 2: Navigation & Sidebar Styling
1. Add active state styling for sidebar links
2. Enhance visual hierarchy in navigation
3. Test on desktop, tablet, mobile viewports
4. Verify hamburger menu on mobile

### Phase 3: Homepage Content
1. Update Hero section with book-specific content
2. Replace placeholder feature cards with module descriptions
3. Update CTA button text and link
4. Add book-relevant imagery (if available)

### Phase 4: Polish & Validation
1. Run Lighthouse audit for performance
2. Test all routes still work
3. Cross-browser testing
4. Final WCAG compliance check

## Key Design Decisions

### Decision 1: CSS Variables over Swizzling
**Choice**: Use Infima CSS variable overrides as primary theming mechanism
**Rationale**: Lower maintenance burden, survives Docusaurus upgrades, sufficient for styling needs
**Alternatives rejected**: Full component swizzling (too much code to maintain)

### Decision 2: System Font Stack
**Choice**: Use system-ui font stack instead of custom web fonts
**Rationale**: Zero additional load time, native feel on each platform
**Alternatives rejected**: Inter/Source Sans (adds ~50-100KB, marginal benefit)

### Decision 3: 18px Base Font Size
**Choice**: 112.5% (18px) base font size
**Rationale**: Optimal for documentation/long-form reading, exceeds 16px minimum
**Alternatives rejected**: 16px (minimum acceptable but less comfortable)

### Decision 4: Blue Primary Color
**Choice**: Blue palette (#2563eb base) replacing green
**Rationale**: Professional, tech-appropriate, good contrast ratios achievable
**Alternatives rejected**: Keep green (too generic), purple (less professional for technical content)

## Complexity Tracking

> No constitution violations requiring justification. Implementation uses standard Docusaurus patterns.

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Color contrast failures | Medium | High | Use WebAIM checker during development |
| Breaking existing styles | Low | Medium | Test all pages after each change |
| Mobile layout issues | Medium | Medium | Test with DevTools device emulation |
| Performance regression | Low | Medium | Run Lighthouse before/after |

## Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| @docusaurus/core | 3.9.2 | Site framework (existing) |
| @docusaurus/preset-classic | 3.9.2 | Classic theme with Infima |
| React | 19.0.0 | Component framework (existing) |
| TypeScript | 5.6.2 | Type safety (existing) |

## Success Metrics

| Metric | Target | How to Measure |
|--------|--------|----------------|
| Contrast ratio | ≥4.5:1 | WebAIM Contrast Checker |
| LCP | <2.5s | Lighthouse |
| FID | <100ms | Lighthouse |
| CLS | <0.1 | Lighthouse |
| Route preservation | 100% | Manual navigation test |
| Mobile usability | Pass | Lighthouse Mobile |

## Next Steps

Run `/sp.tasks` to generate implementation tasks from this plan.
