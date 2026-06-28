use core::marker::PhantomData;

/// All compile-time coordinate-frame tags implement this trait.
///
/// Frame tags are zero-sized types. They never exist at runtime, but they make
/// invalid spatial algebra expressions fail during type checking.
pub trait Frame: 'static {}

/// Built-in inertial world frame.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct WorldFrame;

impl Frame for WorldFrame {}

/// Generated link frame tag.
///
/// A code generator can assign each robot link a unique `ID`, giving types like
/// `LinkFrame<3>` that are distinct at compile time.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LinkFrame<const ID: usize>;

impl<const ID: usize> Frame for LinkFrame<ID> {}

/// Marker carried by typed spatial quantities.
pub(crate) type FrameMarker<F> = PhantomData<fn() -> F>;
