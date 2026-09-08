# Architecture

Reserved for the vehicle architecture and component diagrams.

The skeleton has application, feature module, driver, and shared layers.
The application links feature modules; feature modules link drivers and shared
utilities; drivers link shared utilities. Shared code has no upward dependencies.
