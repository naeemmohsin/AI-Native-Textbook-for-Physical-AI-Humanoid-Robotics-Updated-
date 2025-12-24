# CSS Variable Contract: Docusaurus UI Theme

**Feature**: 005-docusaurus-ui-upgrade
**Date**: 2024-12-24

## Overview

This document defines the CSS custom property interface for the Docusaurus theme. All visual customizations are made by overriding these variables in `src/css/custom.css`.

---

## Color Palette Contract

### Primary Color (7 Shades Required)

```css
:root {
  /* Primary color - used for links, buttons, accents */
  --ifm-color-primary: #2563eb;           /* Base blue */
  --ifm-color-primary-dark: #1d4ed8;      /* Hover state */
  --ifm-color-primary-darker: #1e40af;    /* Active state */
  --ifm-color-primary-darkest: #1e3a8a;   /* Pressed state */
  --ifm-color-primary-light: #3b82f6;     /* Subtle highlight */
  --ifm-color-primary-lighter: #60a5fa;   /* Badge background */
  --ifm-color-primary-lightest: #93c5fd;  /* Lightest accent */
}
```

### Dark Mode Overrides

```css
[data-theme='dark'] {
  --ifm-color-primary: #60a5fa;           /* Lighter for dark bg */
  --ifm-color-primary-dark: #3b82f6;
  --ifm-color-primary-darker: #2563eb;
  --ifm-color-primary-darkest: #1d4ed8;
  --ifm-color-primary-light: #93c5fd;
  --ifm-color-primary-lighter: #bfdbfe;
  --ifm-color-primary-lightest: #dbeafe;
}
```

### Contrast Requirements

| Text/Element | Background | Required Ratio | Target Ratio |
|--------------|------------|----------------|--------------|
| Body text | Page background | 4.5:1 | 7:1 |
| Primary on white | #ffffff | 4.5:1 | 4.5:1+ |
| Primary on dark | #1b1b1d | 4.5:1 | 4.5:1+ |
| Large headings | Page background | 3:1 | 4.5:1 |
| UI components | Adjacent colors | 3:1 | 3:1+ |

---

## Typography Contract

### Base Typography

```css
:root {
  /* Font families */
  --ifm-font-family-base: system-ui, -apple-system, BlinkMacSystemFont,
    'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
  --ifm-font-family-monospace: 'SFMono-Regular', Menlo, Monaco,
    Consolas, 'Liberation Mono', 'Courier New', monospace;

  /* Font sizing - 18px base for documentation readability */
  --ifm-font-size-base: 112.5%;           /* 18px (relative to 16px default) */
  --ifm-code-font-size: 90%;              /* Slightly smaller code */

  /* Line heights */
  --ifm-line-height-base: 1.6;            /* Optimal for long-form reading */
  --ifm-heading-line-height: 1.25;        /* Tighter for headings */

  /* Spacing */
  --ifm-paragraph-margin-bottom: 1rem;
}
```

### Heading Scale

```css
:root {
  --ifm-h1-font-size: 2.25rem;            /* 40.5px */
  --ifm-h2-font-size: 1.75rem;            /* 31.5px */
  --ifm-h3-font-size: 1.375rem;           /* 24.75px */
  --ifm-h4-font-size: 1.125rem;           /* 20.25px */
  --ifm-h5-font-size: 1rem;               /* 18px */
  --ifm-h6-font-size: 0.875rem;           /* 15.75px */
}
```

---

## Layout Contract

### Content Width

```css
:root {
  /* Optimal reading width: 65-75 characters */
  --ifm-container-width: 1400px;          /* Max container */
  --ifm-container-width-xl: 1400px;

  /* Content area constrained for readability */
  --doc-sidebar-width: 300px;
  --doc-content-max-width: 75ch;          /* ~75 characters */
}
```

### Spacing System

```css
:root {
  --ifm-spacing-horizontal: 1rem;
  --ifm-spacing-vertical: 1rem;

  /* Global spacing scale */
  --ifm-global-spacing: 1rem;
  --ifm-global-radius: 0.4rem;            /* Border radius */
}
```

---

## Component-Specific Contracts

### Navbar

```css
:root {
  --ifm-navbar-height: 3.75rem;           /* 60px */
  --ifm-navbar-background-color: var(--ifm-background-color);
  --ifm-navbar-shadow: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
  --ifm-navbar-padding-horizontal: 1rem;
  --ifm-navbar-padding-vertical: 0.5rem;
}
```

### Sidebar

```css
:root {
  --ifm-menu-link-padding-horizontal: 0.75rem;
  --ifm-menu-link-padding-vertical: 0.375rem;

  /* Active item indicator */
  --sidebar-active-border-width: 3px;
  --sidebar-active-border-color: var(--ifm-color-primary);
}

/* Custom: Active state styling */
.menu__link--active {
  font-weight: 600;
  border-left: var(--sidebar-active-border-width) solid var(--sidebar-active-border-color);
  padding-left: calc(var(--ifm-menu-link-padding-horizontal) - var(--sidebar-active-border-width));
}
```

### Footer

```css
.footer {
  --ifm-footer-background-color: #1b1b1d;
  --ifm-footer-color: #e3e3e3;
  --ifm-footer-link-color: #93c5fd;
  --ifm-footer-padding-vertical: 2rem;
  --ifm-footer-padding-horizontal: 1rem;
}
```

### Code Blocks

```css
:root {
  --ifm-code-padding-horizontal: 0.25rem;
  --ifm-code-padding-vertical: 0.1rem;
  --ifm-code-border-radius: 0.25rem;

  /* Inline code */
  --ifm-code-background: rgba(0, 0, 0, 0.05);

  /* Code block */
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.1);
}

[data-theme='dark'] {
  --ifm-code-background: rgba(255, 255, 255, 0.1);
  --docusaurus-highlighted-code-line-bg: rgba(255, 255, 255, 0.1);
}
```

---

## Responsive Breakpoints

```css
/* Docusaurus built-in breakpoints (do not modify) */
/* Reference only - these are not CSS variables */

@media (min-width: 997px) {
  /* Desktop: Full sidebar visible */
}

@media (min-width: 768px) and (max-width: 996px) {
  /* Tablet: Collapsible sidebar */
}

@media (max-width: 767px) {
  /* Mobile: Hamburger menu */
}
```

---

## Validation Checklist

Before deploying, verify:

- [ ] Primary colors generate 7 valid shades
- [ ] All text meets 4.5:1 contrast ratio
- [ ] Font size base is at least 16px (100%)
- [ ] Line height is between 1.4 and 2.0
- [ ] Dark mode colors are distinct from light mode
- [ ] Sidebar active state is clearly visible
- [ ] Code blocks are readable in both themes
