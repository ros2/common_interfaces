Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

Contributors must sign-off each commit by adding a `Signed-off-by: ...`
line to commit messages to certify that they have the right to submit
the code they are contributing to the project according to the
[Developer Certificate of Origin (DCO)](https://developercertificate.org/).


## Contributing new messages and or packages

To be accepted into `common_interfaces` a package needs to have been API reviewed and be in active use in a non trivial portion of the ROS ecosystem.
It's really supposed to represent messages which are commonly used.

On the way to becoming a member of `common_interfaces` please release a message-only package and make it available to the community.
Once it has matured, been reviewed, tested, and possibly iterated upon by early adopters, then it can be promoted to be a member of the `common_interfaces` metapackage.
