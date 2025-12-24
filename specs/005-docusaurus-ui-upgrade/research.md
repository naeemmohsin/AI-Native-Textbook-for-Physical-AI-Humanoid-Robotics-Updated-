# Research: Docusaurus UI Upgrade

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2024-12-24
**Status**: Complete

## Research Questions

1. How to customize Docusaurus 3.x theming via CSS variables?
2. When and how to use component swizzling?
3. What are WCAG AA color contrast requirements?
4. What typography settings optimize readability for documentation?

---

## 1. Docusaurus Theming with Infima CSS Variables

### Decision
Use Infima CSS variable overrides in `src/css/custom.css` as the primary theming mechanism. Avoid swizzling for basic styling changes.

### Rationale
- Docusaurus 3.x uses Infima as its styling framework with `--ifm-*` prefixed variables
- CSS variable overrides are stable across Docusaurus upgrades
- No maintenance burden compared to swizzled components
- Docusaurus-specific variables use `--docusaurus-*` prefix

### Key Implementation Details

**Variable Categories:**
- `--ifm-color-primary-*`: Primary color palette (7 shades needed)
- `--ifm-font-*`: Typography settings
- `--ifm-spacing-*`: Spacing and layout
- `--ifm-navbar-*`: Navbar-specific styling
- `--ifm-footer-*`: Footer-specific styling

**Scoping Consideration:**
Some variables must be scoped to specific selectors, not `:root`. For example:
```css
/* Won't work in :root */
.footer { --ifm-footer-background-color: #303846; }
```

**Color Generation:**
Use [Docusaurus color generator](https://docusaurus.io/docs/styling-layout#styling-your-site-with-infima) or ColorBox to generate 7 shades for primary color.

### Alternatives Considered
- **Direct CSS overrides on Infima classes**: Rejected - classes are implementation details, may change
- **Full theme package**: Rejected - overkill for styling-only changes
- **Swizzling theme components for styling**: Rejected - higher maintenance burden

### Sources
- [Docusaurus Styling and Layout](https://docusaurus.io/docs/styling-layout)
- [Docusaurus.community CSS Variables Guide](https://docusaurus.community/knowledge/design/css/variables/)
- [Infima CSS Variables Gist](https://gist.github.com/abhigyantrips/b828ca46b2460c6699c73e0162f6be80)

---

## 2. Component Swizzling Strategy

### Decision
Use swizzling sparingly - only for homepage components that need structural changes. Prefer wrapping over ejecting.

### Rationale
- Ejecting copies internal code that must be maintained through upgrades
- Wrapping preserves core functionality while adding enhancements
- Homepage components (Hero, HomepageFeatures) need content replacement, not just styling

### When to Swizzle

| Component | Action | Reason |
|-----------|--------|--------|
| `HomepageFeatures` | Eject | Replace placeholder content with book-specific features |
| `src/pages/index.tsx` | Edit directly | Already a custom page, not a theme component |
| Navbar/Sidebar | CSS only | Built-in components work well, just need styling |
| DocItem | Wrap (if needed) | Only if adding custom elements to doc pages |

### Swizzle Commands
```bash
# List available components
npm run swizzle @docusaurus/theme-classic -- --list

# Wrap a component (safer)
npm run swizzle @docusaurus/theme-classic ComponentName -- --wrap

# Eject a component (full control)
npm run swizzle @docusaurus/theme-classic ComponentName -- --eject
```

### Alternatives Considered
- **Swizzle all visual components**: Rejected - excessive maintenance burden
- **Create custom theme package**: Rejected - unnecessary complexity for this scope

### Sources
- [Docusaurus Swizzling Guide](https://docusaurus.io/docs/swizzling)
- [Custom Navbar Items Discussion](https://github.com/facebook/docusaurus/issues/7227)

---

## 3. WCAG AA Color Contrast Requirements

### Decision
Target WCAG 2.1 AA compliance with minimum 4.5:1 contrast for body text and 3:1 for large text and UI components.

### Rationale
- 4.5:1 compensates for vision loss equivalent to 20/40 vision
- AA is the widely accepted standard; AAA (7:1) is ideal but not always achievable
- WCAG 2.1 adds non-text contrast requirements for UI components

### Requirements Summary

| Element | Minimum Ratio | Notes |
|---------|---------------|-------|
| Body text | 4.5:1 | Normal size text (<18pt) |
| Large text | 3:1 | 18pt+ or 14pt bold |
| UI components | 3:1 | Buttons, form borders, icons |
| Focus indicators | 3:1 | Against adjacent colors |

### Implementation Approach
1. Select primary colors that meet 4.5:1 against both white and dark backgrounds
2. Test all color combinations with WebAIM Contrast Checker
3. Ensure focus states are visible with 3:1 contrast
4. Account for anti-aliasing reducing perceived contrast in thin fonts

### Testing Tools
- [WebAIM Contrast Checker](https://webaim.org/resources/contrastchecker/)
- Browser DevTools Accessibility panel
- Lighthouse accessibility audit

### Alternatives Considered
- **AAA compliance (7:1)**: Would significantly limit color palette; AA is sufficient
- **Skip WCAG compliance**: Rejected - accessibility is a core requirement

### Sources
- [W3C WCAG 2.1 Contrast Minimum](https://www.w3.org/WAI/WCAG21/Understanding/contrast-minimum.html)
- [MDN Color Contrast Guide](https://developer.mozilla.org/en-US/docs/Web/Accessibility/Guides/Understanding_WCAG/Perceivable/Color_contrast)
- [WebAIM Contrast Article](https://webaim.org/articles/contrast/)

---

## 4. Typography Best Practices for Documentation

### Decision
Use 18px base font size, 1.6 line height, and 65-75 character line length for optimal documentation readability.

### Rationale
- 16px is minimum; 18px+ recommended for text-heavy sites like documentation
- 1.5-1.6 line height is the sweet spot for longform reading
- 45-90 characters per line is the standard range; 65-75 is optimal

### Recommended Settings

| Property | Value | CSS Variable |
|----------|-------|--------------|
| Base font size | 18px | `--ifm-font-size-base` |
| Line height (body) | 1.6 | `--ifm-line-height-base` |
| Line height (headings) | 1.25 | Custom |
| Paragraph spacing | 1rem | `--ifm-paragraph-margin-bottom` |
| Max content width | 75ch | `--ifm-container-width` |

### Font Stack Recommendations
```css
--ifm-font-family-base: system-ui, -apple-system, BlinkMacSystemFont,
  'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
--ifm-font-family-monospace: 'SFMono-Regular', Menlo, Monaco,
  Consolas, 'Liberation Mono', 'Courier New', monospace;
```

### Code Block Typography
- Slightly smaller than body (0.9em or ~16px)
- Maintain 1.4-1.5 line height for readability
- Ensure syntax highlighting colors meet contrast requirements

### Responsive Considerations
- Use `rem` units for scalability
- Allow text resizing to 200% per WCAG
- Consider slightly smaller base (16px) on mobile if needed

### Alternatives Considered
- **Custom web fonts (Inter, Source Sans)**: Could enhance aesthetics but adds load time; system fonts are sufficient
- **Smaller font size (16px)**: Would work but 18px is more comfortable for extended reading

### Sources
- [Learn UI Design Font Size Guidelines](https://www.learnui.design/blog/mobile-desktop-website-font-size-guidelines.html)
- [U.S. Web Design System Typography](https://designsystem.digital.gov/components/typography/)
- [Justinmind Line Spacing Guide](https://www.justinmind.com/blog/best-ux-practices-for-line-spacing/)

---

## Summary of Technical Decisions

| Area | Decision | Confidence |
|------|----------|------------|
| Theming approach | Infima CSS variables | High |
| Swizzling | Minimal - homepage only | High |
| Color contrast | WCAG AA (4.5:1 minimum) | High |
| Base font size | 18px | High |
| Line height | 1.6 | High |
| Max line length | 75ch | High |

## Open Questions (None)

All technical questions have been resolved through research.
