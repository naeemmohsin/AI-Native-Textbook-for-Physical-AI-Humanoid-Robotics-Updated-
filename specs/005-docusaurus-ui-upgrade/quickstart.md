# Quickstart: Docusaurus UI Upgrade

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2024-12-24

## Prerequisites

- Node.js 20+
- Existing `frontend_book` Docusaurus project
- Git on branch `005-docusaurus-ui-upgrade`

## Quick Setup

```bash
# Navigate to project
cd frontend_book

# Install dependencies (if needed)
npm install

# Start development server
npm start
```

## File Locations

| File | Purpose |
|------|---------|
| `src/css/custom.css` | Theme variable overrides |
| `src/pages/index.tsx` | Homepage layout |
| `src/components/HomepageFeatures/index.tsx` | Feature cards |
| `docusaurus.config.ts` | Site configuration |

## Implementation Order

### Step 1: Typography & Colors (custom.css)

Update `src/css/custom.css` with new variables:

```css
:root {
  /* Typography */
  --ifm-font-size-base: 112.5%;  /* 18px */
  --ifm-line-height-base: 1.6;

  /* Primary color palette */
  --ifm-color-primary: #2563eb;
  --ifm-color-primary-dark: #1d4ed8;
  --ifm-color-primary-darker: #1e40af;
  --ifm-color-primary-darkest: #1e3a8a;
  --ifm-color-primary-light: #3b82f6;
  --ifm-color-primary-lighter: #60a5fa;
  --ifm-color-primary-lightest: #93c5fd;
}
```

### Step 2: Dark Mode Colors

Add dark mode overrides in same file:

```css
[data-theme='dark'] {
  --ifm-color-primary: #60a5fa;
  --ifm-color-primary-dark: #3b82f6;
  /* ... rest of dark palette */
}
```

### Step 3: Sidebar Active State

Add custom sidebar styling:

```css
.menu__link--active {
  font-weight: 600;
  border-left: 3px solid var(--ifm-color-primary);
}
```

### Step 4: Homepage Update

Edit `src/pages/index.tsx`:
- Update CTA button text
- Replace placeholder link

Edit `src/components/HomepageFeatures/index.tsx`:
- Replace Docusaurus placeholders with book modules

### Step 5: Test & Validate

```bash
# Run development server
npm start

# Check specific pages
# - Homepage: http://localhost:3000
# - Docs: http://localhost:3000/docs/intro
# - Dark mode toggle

# Build for production test
npm run build
npm run serve
```

## Validation Commands

```bash
# Type check
npm run typecheck

# Build (catches errors)
npm run build

# Lighthouse audit (after build)
npx lighthouse http://localhost:3000 --view
```

## Key Acceptance Criteria

| Criteria | How to Verify |
|----------|---------------|
| Font size ≥16px | DevTools → Computed Styles |
| Contrast ≥4.5:1 | WebAIM Contrast Checker |
| Mobile responsive | DevTools → Device Toolbar |
| Dark mode works | Toggle switch in navbar |
| Routes preserved | Click all navigation links |
| LCP <2.5s | Lighthouse Performance |

## Common Issues

### Variables not applying
- Check selector specificity
- Some variables need scoped selectors (e.g., `.footer`)

### Dark mode colors wrong
- Ensure `[data-theme='dark']` selector is used
- Check for conflicting `:root` declarations

### Build fails
- Run `npm run typecheck` for TS errors
- Check for missing imports in modified components

## Resources

- [Docusaurus Styling Guide](https://docusaurus.io/docs/styling-layout)
- [Infima CSS Variables](https://infima.dev/)
- [WebAIM Contrast Checker](https://webaim.org/resources/contrastchecker/)
