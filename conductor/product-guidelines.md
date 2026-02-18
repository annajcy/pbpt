# Product Guidelines
These guidelines define the standards for code, documentation, and communication within the PBPT project.

# Documentation
- **Explanatory Focus:** Prioritize explaining the mathematical derivations and logic behind implementations. Focus on the *why* rather than just the *how*.
- **Code Comments:** Use sparingly. Add comments when explaining complex logic or why a specific approach was chosen.

# Naming Conventions
- **Classes:** Use PascalCase (e.g., `PathIntegrator`).
- **Variables & Functions:** Use snake_case (e.g., `ray_intersect`).
- **Namespaces:** Use lowercase (e.g., `pbpt::math`).

# Development Workflow
- **Conventional Commits:** Use prefixes like `feat:`, `fix:`, `refactor:`, `docs:` in commit messages to clearly categorize changes.
- **Error Handling:** Use C++ exceptions for fatal initialization errors or unexpected state failures.

# Quality Assurance
- **Unit Testing:** Maintain high test coverage for individual components using GoogleTest.
- **Integration Testing:** Regularly verify full-scene rendering to ensure components interact correctly.
