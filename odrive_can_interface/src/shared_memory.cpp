#include "odrive_can_interface/shared_memory.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
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
	if (!cmd_if_segment_.open())
	{
		return false;
	}

	if (!state_segment_.open())
	{
		cmd_if_segment_.close();
		return false;
	}

	if (!control_segment_.open())
	{
		cmd_if_segment_.close();
		state_segment_.close();
		return false;
	}

	if (auto *cmd = cmd_if())
	{
		std::memset(cmd, 0, sizeof(HwiCommandIfBlock));
	}
	if (auto *st = state())
	{
		std::memset(st, 0, sizeof(HwiStateBlock));
	}
	if (auto *ctrl = control())
	{
		std::memset(ctrl, 0, sizeof(HshControlBlock));
	}
	return true;
}

bool SharedMemoryInterface::write_state(const HwiStateBlock &state_data)
{
	if (!ready())
	{
		return false;
	}

	HwiStateBlock *st = state();
	if (!st)
	{
		return false;
	}

	std::memcpy(st, &state_data, sizeof(HwiStateBlock));
	if (msync(st, sizeof(HwiStateBlock), MS_SYNC) != 0)
	{
		std::fprintf(stderr, "ERROR: failed to msync state segment, errno=%s\n", std::strerror(errno));
		return false;
	}

	return true;
}

bool SharedMemoryInterface::write_cmd_if(const HwiCommandIfBlock &command_if_data)
{
	if (!ready())
	{
		return false;
	}

	HwiCommandIfBlock *cmd = cmd_if();
	if (!cmd)
	{
		return false;
	}

	std::memcpy(cmd, &command_if_data, sizeof(HwiCommandIfBlock));
	if (msync(cmd, sizeof(HwiCommandIfBlock), MS_SYNC) != 0)
	{
		std::fprintf(stderr, "ERROR: failed to msync cmd_if segment, errno=%s\n", std::strerror(errno));
		return false;
	}

	return true;
}

bool SharedMemoryInterface::write_control(const HshControlBlock &control_data)
{
	if (!ready())
	{
		return false;
	}

	HshControlBlock *ctrl = control();
	if (!ctrl)
	{
		return false;
	}

	std::memcpy(ctrl, &control_data, sizeof(HshControlBlock));
	if (msync(ctrl, sizeof(HshControlBlock), MS_SYNC) != 0)
	{
		std::fprintf(stderr, "ERROR: failed to msync control segment, errno=%s\n", std::strerror(errno));
		return false;
	}

	return true;
}

void SharedMemoryInterface::close()
{
	cmd_if_segment_.close();
	state_segment_.close();
	control_segment_.close();
}

bool SharedMemoryInterface::ready() const noexcept
{
	return cmd_if_segment_.is_open() && state_segment_.is_open() && control_segment_.is_open();
}

HwiCommandIfBlock *SharedMemoryInterface::cmd_if() noexcept
{
	return cmd_if_segment_.as<HwiCommandIfBlock>();
}

const HwiCommandIfBlock *SharedMemoryInterface::cmd_if() const noexcept
{
	return cmd_if_segment_.as<const HwiCommandIfBlock>();
}

HwiStateBlock *SharedMemoryInterface::state() noexcept
{
	return state_segment_.as<HwiStateBlock>();
}

const HwiStateBlock *SharedMemoryInterface::state() const noexcept
{
	return state_segment_.as<const HwiStateBlock>();
}

HshControlBlock *SharedMemoryInterface::control() noexcept
{
	return control_segment_.as<HshControlBlock>();
}

const HshControlBlock *SharedMemoryInterface::control() const noexcept
{
	return control_segment_.as<const HshControlBlock>();
}

} // namespace odrive_can_interface
