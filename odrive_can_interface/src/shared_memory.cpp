#include "odrive_can_interface/shared_memory.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <utility>
#include <cstdio>

namespace odrive_can_interface
{

SharedMemorySegment::SharedMemorySegment(std::string name, std::size_t size)
	: name_(std::move(name)), size_(size)
{
}

SharedMemorySegment::SharedMemorySegment(SharedMemorySegment &&other) noexcept
{
	*this = std::move(other);
}

SharedMemorySegment &SharedMemorySegment::operator=(SharedMemorySegment &&other) noexcept
{
	if (this == &other)
		return *this;

	close();

	name_ = std::move(other.name_);
	size_ = other.size_;
	fd_ = other.fd_;
	ptr_ = other.ptr_;

	other.size_ = 0;
	other.fd_ = -1;
	other.ptr_ = nullptr;

	return *this;
}

SharedMemorySegment::~SharedMemorySegment()
{
	close();
}

bool SharedMemorySegment::open(bool create_if_missing)
{
	close();

	int flags = O_RDWR;
	if (create_if_missing)
	{
		flags |= O_CREAT;
	}

	fd_ = shm_open(name_.c_str(), flags, 0666);
	if (fd_ < 0)
	{
        std::fprintf(stderr, "ERROR: could not create FD, errno= %s\n",std::strerror(errno));
		return false;
	}

	if (ftruncate(fd_, static_cast<off_t>(size_)) != 0)
	{
		::close(fd_);
		fd_ = -1;
        std::fprintf(stderr, "ERROR: could ftruncate FD, errno=%s\n", std::strerror(errno));
		return false;
	}

	void *addr = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
	if (addr == MAP_FAILED)
	{
		::close(fd_);
		fd_ = -1;
        std::fprintf(stderr, "ERROR: could not create ADDRESS, errno=%s\n", std::strerror(errno));
		return false;
	}

	ptr_ = addr;
	return true;
}

void SharedMemorySegment::close()
{
	if (ptr_)
	{
		munmap(ptr_, size_);
		ptr_ = nullptr;
	}

	if (fd_ >= 0)
	{
		::close(fd_);
		fd_ = -1;
	}
}

void SharedMemorySegment::reset()
{
	close();
	name_.clear();
	size_ = 0;
}

SharedMemoryInterface::~SharedMemoryInterface()
{
	close();
}

bool SharedMemoryInterface::open()
{
	if (!command_segment_.open())
	{
		return false;
	}

	if (!state_segment_.open())
	{
		command_segment_.close();
		return false;
	}

	if (auto *cmd = command())
	{
		std::memset(cmd, 0, sizeof(ShmCommand));
	}
	if (auto *st = state())
	{
		std::memset(st, 0, sizeof(ShmState));
	}
	return true;
}

bool SharedMemoryInterface::write_state(const ShmState &state_data)
{
	if (!ready())
	{
		return false;
	}

	ShmState *st = state();
	if (!st)
	{
		return false;
	}

	std::memcpy(st, &state_data, sizeof(ShmState));
	if (msync(st, sizeof(ShmState), MS_SYNC) != 0)
	{
		std::fprintf(stderr, "ERROR: failed to msync state segment, errno=%s\n", std::strerror(errno));
		return false;
	}

	return true;
}

bool SharedMemoryInterface::write_cmd(const ShmCommand &command_data)
{
	if (!ready())
	{
		return false;
	}

	ShmCommand *cmd = command();
	if (!cmd)
	{
		return false;
	}

	std::memcpy(cmd, &command_data, sizeof(ShmCommand));
	if (msync(cmd, sizeof(ShmCommand), MS_SYNC) != 0)
	{
		std::fprintf(stderr, "ERROR: failed to msync command segment, errno=%s\n", std::strerror(errno));
		return false;
	}

	return true;
}

void SharedMemoryInterface::close()
{
	command_segment_.close();
	state_segment_.close();
}

bool SharedMemoryInterface::ready() const noexcept
{
	return command_segment_.is_open() && state_segment_.is_open();
}

ShmCommand *SharedMemoryInterface::command() noexcept
{
	return command_segment_.as<ShmCommand>();
}

const ShmCommand *SharedMemoryInterface::command() const noexcept
{
	return command_segment_.as<const ShmCommand>();
}

ShmState *SharedMemoryInterface::state() noexcept
{
	return state_segment_.as<ShmState>();
}

const ShmState *SharedMemoryInterface::state() const noexcept
{
	return state_segment_.as<const ShmState>();
}

} // namespace odrive_can_interface

