# Contributing

First off, thank you for considering contributing to Jupyter-ROS 🥳. It's people like you that make Jupyter-ROS such a great tool.

Following these guidelines helps to communicate that you respect the time of the developers managing and developing this open source project. In return, they should reciprocate that respect in addressing your issue, assessing changes, and helping you finalize your pull requests.

Jupyter-ROS is an open source project and we love to receive contributions from our community — you! There are many ways to contribute. For example, you can

- write a new tutorial or a blog post
- improve the documentation or the existing examples
- submit bug reports or feature requests
- write code to incorporate into Jupyter-ROS itself

## Ground Rules

We welcome all kinds of contribution and value them highly. We pledge to treat everyone's contribution fairly and with respect, and we are here to bring awesome pull requests over the finish line.

Please note that we adhere to the [Python Community Code of Conduct](https://www.python.org/psf/codeofconduct/) and by contributing to this project you also agree to follow the same guidelines.

## Your First Contribution

Working on your first Pull Request? Here are some resources to help you get started:

- [First Timers Only](https://www.firsttimersonly.com/)
- [Make a Pull Request](https://makeapullrequest.com/)
- [How to Contribute to an Open Source Project on GitHub](https://egghead.io/courses/how-to-contribute-to-an-open-source-project-on-github)

At this point, you're ready to make your changes! Feel free to ask for help; everyone is a beginner at first 😸.

If a maintainer asks you to "rebase" your PR, they're saying that a lot of code has changed and that you need to update your branch so that it's easier to merge.

## Getting Started

1. Create your own fork of the code.
2. Do the changes in your fork.
3. If your changes only involve spelling or grammar fixes, move to step 7.
4. Test your changes in a clean environment and update installation instructions and dependencies as needed.
5. When adding new features, make sure to update the documentation and provide an example under _notebooks/_.
6. New notebooks
   - Remove all output.
   - Remove unnecessary cells.
   - Include a descriptive title.
   - Specify ROS version in the notebook name, "_ROS Turtlesim.ipynb_" vs "_ROS**2** Turtlesim.ipynb_"
   - Any additional steps the user needs to take to run all the cells in the notebook should be clearly stated in markdown cells.
7. If you are happy with your changes, create a pull request.

## How to Report a Bug

### Security

If you find a security vulnerability, do NOT open an issue. Email [w.vollprecht@gmail.com](mailto:w.vollprecht@gmail.com) instead. In order to determine whether you are dealing with a security issue, ask yourself these two questions:

- Can I access something that's not mine, or something I shouldn't have access to?
- Can I disable something for other people?

If the answer to either of those two questions are "yes", then you're probably dealing with a security issue. Note that even if you answer "no" to both questions, you may still be dealing with a security issue, so if you're unsure, just email us.

### Other bugs

When filing an issue, make sure to answer these five questions:

1. What version of _jupyros_ are you using?
2. What operating system and processor architecture are you using?
3. What did you do?
4. What did you expect to see?
5. What did you see instead?

General questions should be handled through [Gitter](https://gitter.im/RoboStack/Lobby) instead of the issue tracker. The maintainers there will answer or ask you to file an issue if you've tripped over a bug.

## How to Suggest a Feature or Enhancement

If you find yourself wishing for a feature that doesn't exist in Jupyter-ROS, you are probably not alone. There are bound to be others out there with similar needs. Many of the features that Jupyter-ROS has today have been added because our users saw the need. Open an issue on our [issues list](https://github.com/RoboStack/jupyter-ros/issues) on GitHub which includes the following:

1. A description of the feature you would like to see
2. Why do you need it?
3. How should it work?

## Code Review Process

Any change to resources in this repository must be through pull requests. This applies to all changes to documentation, code, binary files, etc. No pull request can be merged without being reviewed.

The core team looks at Pull Requests on a regular basis and provides feedback after each review. Once feedback has been given, we expect responses within three weeks. After the three weeks have elapsed, we may close the pull request if it isn't showing any activity.

A pull request will be merged once all the feedback has been addressed and there are no objections by any of the committers.

## Community

You can chat with the core team on [gitter.im/RoboStack](https://gitter.im/RoboStack/Lobby). We try to answer all questions within 48 hours.
