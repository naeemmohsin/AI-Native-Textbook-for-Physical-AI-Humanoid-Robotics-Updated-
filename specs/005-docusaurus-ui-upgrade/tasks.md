# Tasks: Docusaurus UI Upgrade

**Input**: Design documents from `/specs/005-docusaurus-ui-upgrade/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/css-variables.md

**Tests**: Manual visual testing, Lighthouse audits, WebAIM Contrast Checker (no automated test tasks - visual verification)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

All paths relative to `frontend_book/`:
- `src/css/custom.css` - Theme CSS variables
- `src/pages/index.tsx` - Homepage
- `src/components/HomepageFeatures/` - Feature cards
- `docusaurus.config.ts` - Site configuration

---

## Phase 1: Setup

**Purpose**: Verify development environment and baseline

- [x] T001 Verify Node.js 20+ and npm installed, run `npm install` in frontend_book/
- [x] T002 Run `npm start` to verify dev server works at http://localhost:3000
- [ ] T003 Run baseline Lighthouse audit and save results for comparison in specs/005-docusaurus-ui-upgrade/baseline-lighthouse.json

**Checkpoint**: Development environment ready

---

## Phase 2: Foundational (CSS Variable Infrastructure)

**Purpose**: Establish theme foundation that ALL user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Add typography variables (font-size-base: 112.5%, line-height-base: 1.6) in frontend_book/src/css/custom.css
- [x] T005 [P] Add primary color palette (7 shades from #2563eb) for light mode in frontend_book/src/css/custom.css
- [x] T006 [P] Add dark mode color palette overrides in [data-theme='dark'] selector in frontend_book/src/css/custom.css
- [x] T007 Add spacing and layout variables (container-width, global-spacing) in frontend_book/src/css/custom.css
- [ ] T008 Verify contrast ratios meet WCAG AA (4.5:1) using WebAIM Contrast Checker for both themes

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Reading Experience Enhancement (Priority: P1) MVP

**Goal**: Comfortable, accessible reading experience with proper typography and colors

**Independent Test**: Load any documentation page, verify 18px font size, 1.6 line height, WCAG AA contrast in both light/dark modes

### Implementation for User Story 1

- [x] T009 [US1] Add heading scale variables (h1-h6 font sizes) in frontend_book/src/css/custom.css
- [x] T010 [P] [US1] Add paragraph spacing (--ifm-paragraph-margin-bottom: 1rem) in frontend_book/src/css/custom.css
- [x] T011 [P] [US1] Add code block styling variables (code-font-size, code-background, code-border-radius) in frontend_book/src/css/custom.css
- [x] T012 [US1] Add dark mode code block styling in [data-theme='dark'] selector in frontend_book/src/css/custom.css
- [x] T013 [US1] Add syntax highlighting theme colors (prism themes already configured in docusaurus.config.ts - verify working)
- [ ] T014 [US1] Test reading experience on a documentation page in both light and dark modes
- [ ] T015 [US1] Verify font rendering at 200% browser zoom

**Checkpoint**: User Story 1 complete - reading experience verified independently

---

## Phase 4: User Story 2 - Module Navigation Flow (Priority: P1)

**Goal**: Clear navigation with visible current location and easy module switching

**Independent Test**: Navigate from homepage through modules, verify sidebar highlights current page, previous/next links work

### Implementation for User Story 2

- [x] T016 [US2] Add sidebar active state styling (.menu__link--active with left border indicator) in frontend_book/src/css/custom.css
- [x] T017 [P] [US2] Add sidebar hover state styling (.menu__link:hover) in frontend_book/src/css/custom.css
- [x] T018 [P] [US2] Add navbar styling variables (height, shadow, padding) in frontend_book/src/css/custom.css
- [x] T019 [US2] Add footer styling with scoped variables (.footer selector) in frontend_book/src/css/custom.css
- [ ] T020 [US2] Verify previous/next navigation links appear at bottom of doc pages (built-in Docusaurus feature)
- [ ] T021 [US2] Test navigation flow: homepage → module → chapter → verify breadcrumb and sidebar state
- [ ] T022 [US2] Verify hamburger menu works on mobile viewport (<768px) using DevTools

**Checkpoint**: User Story 2 complete - navigation verified independently

---

## Phase 5: User Story 3 - Homepage Value Proposition (Priority: P2)

**Goal**: Clear book introduction with module highlights and prominent CTA

**Independent Test**: Load homepage, verify title/tagline visible above fold, CTA leads to intro, feature cards describe actual modules

### Implementation for User Story 3

- [x] T023 [US3] Update hero section CTA button text from "Docusaurus Tutorial" to "Start Reading" in frontend_book/src/pages/index.tsx
- [x] T024 [US3] Update hero section CTA link to point to /docs/intro in frontend_book/src/pages/index.tsx
- [x] T025 [US3] Replace FeatureList placeholder content with book modules (ROS 2, NVIDIA Isaac, VLA) in frontend_book/src/components/HomepageFeatures/index.tsx
- [x] T026 [P] [US3] Update feature card descriptions to match actual module content in frontend_book/src/components/HomepageFeatures/index.tsx
- [x] T027 [P] [US3] Add hero banner gradient or solid background style in frontend_book/src/pages/index.module.css
- [x] T028 [US3] Remove or replace default Docusaurus SVG illustrations with book-appropriate icons in frontend_book/src/components/HomepageFeatures/index.tsx
- [ ] T029 [US3] Test homepage loads with book title, tagline, and working CTA above the fold

**Checkpoint**: User Story 3 complete - homepage verified independently

---

## Phase 6: User Story 4 - Responsive Layout Adaptation (Priority: P2)

**Goal**: Consistent experience across desktop, tablet, and mobile viewports

**Independent Test**: Load site at 320px, 768px, 1200px, 2560px widths - verify no horizontal scroll, readable content, accessible navigation

### Implementation for User Story 4

- [x] T030 [US4] Add content max-width constraint (--doc-content-max-width: 75ch) for optimal reading width in frontend_book/src/css/custom.css
- [x] T031 [P] [US4] Add responsive adjustments for mobile font sizes if needed in frontend_book/src/css/custom.css
- [ ] T032 [US4] Test desktop layout (1200px+): sidebar visible, content sized appropriately
- [ ] T033 [US4] Test tablet layout (768-1199px): collapsible sidebar, full content width
- [ ] T034 [US4] Test mobile layout (<768px): hamburger menu, no horizontal scroll
- [ ] T035 [US4] Test device rotation: landscape/portrait reflow without content jumps

**Checkpoint**: User Story 4 complete - responsive layout verified independently

---

## Phase 7: Polish & Validation

**Purpose**: Final quality checks and cross-cutting concerns

- [ ] T036 Run Lighthouse performance audit, verify LCP <2.5s, FID <100ms, CLS <0.1
- [ ] T037 [P] Run Lighthouse accessibility audit, verify score >=90
- [ ] T038 [P] Test all existing documentation routes still work (no broken links)
- [x] T039 Run `npm run build` to verify production build succeeds
- [ ] T040 [P] Cross-browser test: Chrome, Firefox, Safari, Edge (visual spot check)
- [ ] T041 Final WCAG AA contrast verification for all color combinations
- [ ] T042 Document any deviations from plan in specs/005-docusaurus-ui-upgrade/notes.md (if any)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - US1 and US2 (both P1): Can run in parallel after Foundational
  - US3 and US4 (both P2): Can run in parallel after Foundational
- **Polish (Phase 7)**: Depends on all user stories complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational - No dependencies on other stories
- **User Story 4 (P2)**: Can start after Foundational - No dependencies on other stories

### Parallel Opportunities

Within each phase, tasks marked [P] can run in parallel:
- Phase 2: T005, T006 (color palettes)
- Phase 3: T010, T011 (typography details)
- Phase 4: T017, T018 (navigation styling)
- Phase 5: T026, T027 (homepage content)
- Phase 7: T037, T038, T040 (validation tasks)

---

## Parallel Example: Foundational Phase

```bash
# Launch color palette tasks in parallel:
Task: T005 "Add primary color palette (7 shades) for light mode"
Task: T006 "Add dark mode color palette overrides"
```

## Parallel Example: User Stories

```bash
# After Foundational complete, launch P1 stories in parallel:
Task: Phase 3 (US1 - Reading Experience)
Task: Phase 4 (US2 - Navigation Flow)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CSS infrastructure)
3. Complete Phase 3: User Story 1 (Reading Experience)
4. **STOP and VALIDATE**: Reading experience works in both themes
5. Deploy/demo if ready - site is readable and usable

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add User Story 1 → Reading works → Deploy (MVP!)
3. Add User Story 2 → Navigation works → Deploy
4. Add User Story 3 → Homepage updated → Deploy
5. Add User Story 4 → Fully responsive → Deploy
6. Polish → Production-ready → Final Deploy

### Recommended Order (Single Developer)

1. Phase 1: Setup (T001-T003)
2. Phase 2: Foundational (T004-T008)
3. Phase 3: US1 Reading (T009-T015) - Core value first
4. Phase 4: US2 Navigation (T016-T022) - Complete P1 stories
5. Phase 5: US3 Homepage (T023-T029)
6. Phase 6: US4 Responsive (T030-T035)
7. Phase 7: Polish (T036-T042)

---

## Task Summary

| Phase | Task Count | Parallel Tasks | Completed |
|-------|------------|----------------|-----------|
| 1. Setup | 3 | 0 | 2/3 |
| 2. Foundational | 5 | 2 | 4/5 |
| 3. US1 Reading | 7 | 2 | 5/7 |
| 4. US2 Navigation | 7 | 2 | 4/7 |
| 5. US3 Homepage | 7 | 2 | 6/7 |
| 6. US4 Responsive | 6 | 1 | 2/6 |
| 7. Polish | 7 | 3 | 1/7 |
| **Total** | **42** | **12** | **24/42** |

---

## Notes

- [P] tasks = different files or independent operations
- [Story] label maps task to specific user story for traceability
- Each user story is independently testable after completion
- Verify visual changes in browser after each CSS modification
- Commit after each completed task or logical group
- All styling changes go in custom.css - avoid swizzling components

## Implementation Status

**Status**: Implementation Complete (code changes)
**Remaining**: Manual testing/verification tasks (T003, T008, T014-T015, T020-T022, T029, T032-T038, T040-T042)

These remaining tasks require manual browser testing and cannot be automated in this session.
