FROM rezenders/beaglebone-black-ubuntu-openjdk-ros


# Install packages
RUN apt-get update && apt-get install -y \
	vim \
	gradle  \
	&& rm -rf /var/lib/apt/lists/

# Download, install and configure Jason
WORKDIR /
RUN ["/bin/bash", "-c", "git clone https://github.com/jason-lang/jason.git"]
WORKDIR /jason
RUN ["/bin/bash", "-c", "gradle config"]
ENV JASON_HOME=/jason/build
ENV PATH=$JASON_HOME/scripts:$PATH

COPY rosjava_agents/ /rosjava_agents/
WORKDIR /rosjava_agents

CMD ["bash"]
