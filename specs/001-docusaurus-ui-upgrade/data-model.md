# Data Model: Docusaurus UI Upgrade

## Entities

### Theme Configuration
- **Properties**:
  - colorPalette: Object containing light/dark mode color schemes
  - typography: Font families, sizes, weights, line heights
  - spacing: Spacing scale and layout properties
  - breakpoints: Responsive design breakpoints
- **Validation**: Must maintain accessibility contrast ratios (WCAG AA minimum)
- **Relationships**: Applied globally across all documentation pages

### Navigation Hierarchy
- **Properties**:
  - moduleStyling: Visual styling for module-level items
  - chapterStyling: Visual styling for chapter-level items
  - indentation: Visual hierarchy indicators
  - expandCollapse: UI elements for collapsible sections
- **Validation**: Must preserve existing sidebar structure and routing
- **Relationships**: Connected to sidebar configuration (sidebars.js)

### Documentation Pages
- **Properties**:
  - contentArea: Styling for main content area
  - typography: Per-page typography settings
  - layout: Page layout configuration
- **Validation**: Must maintain compatibility with existing Markdown content
- **Relationships**: Inherits from global theme configuration

## State Transitions

### Theme State
- **light**: Default theme state
- **dark**: Dark mode state (user-triggered)
- **transitions**: light ↔ dark (via theme switcher)

## Constraints

- All styling must be compatible with existing Markdown content
- No changes to document IDs or file paths
- No breaking changes to existing routing
- Maintain accessibility standards (WCAG AA)